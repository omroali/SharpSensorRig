#!/usr/bin/env python3
"""Turn SHARP ``/sharp/events`` into activity intervals and per-frame labels.

The session runner records annotation events on ``/sharp/events`` as
``std_msgs/String`` carrying a JSON event row::

  {"event_uuid": …, "session_id": …, "trial_uuid": …, "ros_ns": …,
   "wall_iso": …, "type": …, "payload": {…}}

Action boundaries are ``action_start`` / ``action_end`` / ``action_skipped``:

  * T1 (atomic-action phase): ``trial_uuid`` is null; payload has ``action_id``,
    ``action_name``, ``position``, ``rep``, ``redo``, ``source``.
  * T2 (structured trials): ``trial_uuid`` set; payload has ``action_id``,
    ``action_name``, ``step_index``, ``source`` (and ``variation`` on some steps).
  * T3 (free execution): ``t3_annotation`` marks an action at an instant.

This module pairs starts with their matching ends into non-overlapping
*activities*, then labels every recorded video frame with the activity it falls
inside — the bridge between sensor frames and what the participant was doing.

Usage:
  ros2 run session_recorder session_activities <session_dir>

Writes:
  <session>/activities.yaml          interval list (also used for jumping)
  <session>/labels/<stream>.csv      frame_idx,ts,activity columns per video

The parsing/labelling functions are pure (no ROS) and unit-tested; only
``read_events_from_bag`` imports ``rosbag2_py``.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import glob
import json
import os
import sys

import yaml

EVENTS_TOPIC = "/sharp/events"

# Boundary event types that open / close an activity.
OPEN_TYPES = {"action_start"}
CLOSE_TYPES = {"action_end", "action_skipped"}
POINT_TYPES = {"t3_annotation"}


def _tier_for(trial_uuid) -> str:
    return "T1" if trial_uuid is None else "T2"


def parse_activities(events: list[dict]) -> list[dict]:
    """Pair action boundaries into non-overlapping activities, in time order.

    Robust to out-of-order input (sorted by ``ros_ns``), to a missing trailing
    end (closed at the last event / session end), and to a new start arriving
    while one is still open (the open one is closed at the new start).
    """
    ordered = sorted(
        (e for e in events if isinstance(e, dict) and "ros_ns" in e),
        key=lambda e: (int(e["ros_ns"]), str(e.get("event_uuid", ""))),
    )

    activities: list[dict] = []
    open_activity: dict | None = None

    def _close(ns: int, skipped: bool = False) -> None:
        nonlocal open_activity
        if open_activity is None:
            return
        open_activity["end_ns"] = int(ns)
        open_activity["skipped"] = skipped
        activities.append(open_activity)
        open_activity = None

    for event in ordered:
        etype = str(event.get("type", ""))
        ns = int(event["ros_ns"])
        trial_uuid = event.get("trial_uuid")
        payload = event.get("payload") or {}

        if etype in OPEN_TYPES:
            _close(ns)  # defensive: a dangling start is closed here
            open_activity = {
                "index": len(activities),
                "tier": _tier_for(trial_uuid),
                "trial_uuid": trial_uuid,
                "action_id": payload.get("action_id"),
                "action_name": payload.get("action_name"),
                "position": payload.get("position"),
                "step_index": payload.get("step_index"),
                "rep": payload.get("rep"),
                "redo": bool(payload.get("redo", False)),
                "source": payload.get("source"),
                "variation": payload.get("variation"),
                "start_ns": ns,
                "end_ns": None,
                "skipped": False,
            }
        elif etype in CLOSE_TYPES:
            _close(ns, skipped=(etype == "action_skipped"))
        elif etype in POINT_TYPES:
            activities.append({
                "index": len(activities),
                "tier": "T3",
                "trial_uuid": trial_uuid,
                "action_id": payload.get("action_id"),
                "action_name": payload.get("action_name"),
                "position": None,
                "step_index": None,
                "rep": None,
                "redo": False,
                "source": payload.get("source"),
                "variation": None,
                "point": True,
                "text": payload.get("text"),
                "note": payload.get("note"),
                "start_ns": ns,
                "end_ns": ns,
                "skipped": False,
            })
        elif etype == "session_end":
            _close(ns)

    if open_activity is not None:
        last_ns = int(ordered[-1]["ros_ns"]) if ordered else open_activity["start_ns"]
        _close(last_ns)

    for i, activity in enumerate(activities):
        activity["index"] = i
    return activities


class ActivityIndex:
    """Fast ``activity_at(ns)`` lookup over non-overlapping activities."""

    def __init__(self, activities: list[dict]):
        self._activities = [
            a for a in activities if a.get("end_ns") is not None
        ]
        self._starts = [int(a["start_ns"]) for a in self._activities]

    def activity_at(self, ns: int) -> dict | None:
        ns = int(ns)
        pos = bisect.bisect_right(self._starts, ns) - 1
        if pos < 0:
            return None
        activity = self._activities[pos]
        if ns <= int(activity["end_ns"]):
            return activity
        return None

    def label(self, ns: int) -> dict:
        """Flat label columns for a frame at ``ns`` (empty strings if outside)."""
        activity = self.activity_at(ns)
        if activity is None:
            return {
                "activity_index": "",
                "tier": "",
                "trial_uuid": "",
                "action_id": "",
                "action_name": "",
                "rep": "",
                "skipped": "",
            }
        return {
            "activity_index": activity["index"],
            "tier": activity.get("tier", ""),
            "trial_uuid": activity.get("trial_uuid") or "",
            "action_id": activity.get("action_id") or "",
            "action_name": activity.get("action_name") or "",
            "rep": "" if activity.get("rep") is None else activity["rep"],
            "skipped": activity.get("skipped", False),
        }


def label_video_csv(csv_path: str, index: ActivityIndex) -> list[dict]:
    """Label one colour stream's timestamp CSV (frame_idx,ros_timestamp_ns)."""
    rows = []
    with open(csv_path, newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            try:
                frame_idx = int(row["frame_idx"])
                ts = int(row["ros_timestamp_ns"])
            except (KeyError, TypeError, ValueError):
                continue
            rows.append({"frame_idx": frame_idx, "ros_timestamp_ns": ts, **index.label(ts)})
    return rows


def write_labels(rows: list[dict], out_path: str) -> None:
    fields = [
        "frame_idx", "ros_timestamp_ns", "activity_index", "tier",
        "trial_uuid", "action_id", "action_name", "rep", "skipped",
    ]
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


# ── bag I/O (needs ROS) ─────────────────────────────────────────────────────


def find_bag_dir(session_dir: str) -> str | None:
    candidates = sorted(glob.glob(os.path.join(session_dir, "bag*", "metadata.yaml")))
    return os.path.dirname(candidates[-1]) if candidates else None


def read_events_from_bag(bag_dir: str, topic: str = EVENTS_TOPIC) -> list[dict]:
    """Read and JSON-decode every event row on ``topic`` in the bag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    with open(os.path.join(bag_dir, "metadata.yaml"), "r", encoding="utf-8") as handle:
        info = (yaml.safe_load(handle) or {}).get("rosbag2_bagfile_information", {}) or {}

    types = {}
    for entry in info.get("topics_with_message_count", []) or []:
        metadata = entry.get("topic_metadata", {}) or {}
        if metadata.get("name"):
            types[str(metadata["name"])] = str(metadata.get("type", ""))
    if topic not in types:
        return []

    msg_type = get_message(types[topic])
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id=str(info.get("storage_identifier", "mcap"))),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    try:
        reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    except Exception:
        pass

    events = []
    while reader.has_next():
        name, data, _stamp = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, msg_type)
        raw = getattr(msg, "data", "")
        try:
            events.append(json.loads(raw))
        except (TypeError, ValueError):
            pass
    return events


# ── CLI ─────────────────────────────────────────────────────────────────────


def _summarise(activities: list[dict]) -> str:
    if not activities:
        return "no activities"
    by_tier: dict[str, int] = {}
    for activity in activities:
        by_tier[activity["tier"]] = by_tier.get(activity["tier"], 0) + 1
    parts = ", ".join(f"{tier}: {n}" for tier, n in sorted(by_tier.items()))
    return f"{len(activities)} activities ({parts})"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir")
    parser.add_argument("--topic", default=EVENTS_TOPIC)
    parser.add_argument("--out-dir", default=None,
                        help="Where to write labels (default <session>/labels)")
    args = parser.parse_args(argv)

    session = os.path.abspath(args.session_dir)
    if not os.path.isdir(session):
        print(f"ERROR: not a directory: {session}", file=sys.stderr)
        return 2

    bag_dir = find_bag_dir(session)
    if bag_dir is None:
        print(f"ERROR: no bag*/metadata.yaml under {session}", file=sys.stderr)
        return 2

    events = read_events_from_bag(bag_dir, args.topic)
    print(f"Read {len(events)} events from {args.topic}")
    activities = parse_activities(events)
    print(_summarise(activities))
    if not activities:
        print("Nothing to label — is /sharp/events present in this bag?")
        return 0

    activities_path = os.path.join(session, "activities.yaml")
    with open(activities_path, "w", encoding="utf-8") as handle:
        yaml.safe_dump({"activities": activities}, handle, sort_keys=False)
    print(f"Wrote {activities_path}")

    index = ActivityIndex(activities)
    labels_dir = args.out_dir or os.path.join(session, "labels")
    videos_dir = os.path.join(session, "videos")
    count = 0
    for csv_path in sorted(glob.glob(os.path.join(videos_dir, "*.csv"))):
        stem = os.path.splitext(os.path.basename(csv_path))[0]
        rows = label_video_csv(csv_path, index)
        if not rows:
            continue
        labelled = sum(1 for row in rows if row["activity_index"] != "")
        write_labels(rows, os.path.join(labels_dir, f"{stem}.csv"))
        print(f"  {stem}: {labelled}/{len(rows)} frames labelled")
        count += 1
    print(f"Wrote {count} label files under {labels_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
