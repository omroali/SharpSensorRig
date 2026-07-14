#!/usr/bin/env python3
"""Post-session sanity check for recorded rig sessions.

Compares what a session actually contains against the expected_topics.yaml
manifest that recording_manager wrote at start time:

  - every expected bag topic is present in the bag with a healthy rate
  - regex-expected topics (vicon) matched at least one topic, each healthy
  - every expected video has an .mp4 and a timestamp CSV whose frame rate
    and time span cover the session (catches encoders that died mid-take)
  - rosbag transport-layer message loss is surfaced

Writes <session>/session_report.yaml, prints one PASS/FAIL summary line
last, and exits non-zero on failure so callers can gate on it.

Usage: ros2 run session_recorder session_check <session_dir>
"""

from __future__ import annotations

import glob
import os
import re
import sys
from datetime import datetime

import yaml

# A video/bag stream must span at least this fraction of the session,
# otherwise its recorder died mid-take.
MIN_SPAN_FRACTION = 0.9


def _load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _bag_metadata(session_dir: str) -> tuple[dict, str]:
    """Find the session's bag metadata (bag/, bag_1/, ...)."""
    candidates = sorted(glob.glob(os.path.join(session_dir, "bag*", "metadata.yaml")))
    if not candidates:
        return {}, ""
    return _load_yaml(candidates[-1]), candidates[-1]


def _bag_topic_counts(metadata: dict) -> dict[str, int]:
    info = metadata.get("rosbag2_bagfile_information", {}) or {}
    counts = {}
    for entry in info.get("topics_with_message_count", []) or []:
        name = (entry.get("topic_metadata", {}) or {}).get("name")
        if name:
            counts[str(name)] = int(entry.get("message_count", 0))
    return counts


def _bag_duration_s(metadata: dict) -> float:
    info = metadata.get("rosbag2_bagfile_information", {}) or {}
    return float((info.get("duration", {}) or {}).get("nanoseconds", 0)) / 1e9


def topic_to_filename(topic: str) -> str:
    return topic.strip("/").replace("/", "_")


def _check_rate(
    checks: list, topic: str, count: int, duration: float,
    expected_hz: float | None, min_rate_factor: float, sink: str,
):
    if count <= 0:
        checks.append({
            "topic": topic, "sink": sink, "status": "fail",
            "detail": "no messages recorded",
        })
        return
    if expected_hz is None or duration <= 0:
        checks.append({
            "topic": topic, "sink": sink, "status": "ok",
            "count": count, "detail": "present (no rate threshold)",
        })
        return
    hz = count / duration
    min_hz = expected_hz * min_rate_factor
    status = "ok" if hz >= min_hz else "fail"
    checks.append({
        "topic": topic, "sink": sink, "status": status,
        "count": count, "hz": round(hz, 2),
        "expected_hz": expected_hz, "min_hz": round(min_hz, 2),
        "detail": "" if status == "ok" else f"rate {hz:.2f} Hz below minimum {min_hz:.2f} Hz",
    })


def _check_video(
    checks: list, topic: str, videos_dir: str, duration: float,
    expected_hz: float | None, min_rate_factor: float,
):
    stem = topic_to_filename(topic)
    mp4 = os.path.join(videos_dir, f"{stem}.mp4")
    csv_path = os.path.join(videos_dir, f"{stem}.csv")

    if not os.path.isfile(mp4) or os.path.getsize(mp4) == 0:
        checks.append({
            "topic": topic, "sink": "video", "status": "fail",
            "detail": f"missing or empty video: {mp4}",
        })
        return
    if not os.path.isfile(csv_path):
        checks.append({
            "topic": topic, "sink": "video", "status": "fail",
            "detail": f"missing timestamp CSV: {csv_path}",
        })
        return

    first_ns = last_ns = None
    frames = 0
    with open(csv_path, "r", encoding="utf-8") as handle:
        next(handle, None)  # header: frame_idx,ros_timestamp_ns
        for line in handle:
            parts = line.strip().split(",")
            if len(parts) != 2:
                continue
            frames += 1
            ts = int(parts[1])
            if first_ns is None:
                first_ns = ts
            last_ns = ts

    if frames == 0:
        checks.append({
            "topic": topic, "sink": "video", "status": "fail",
            "detail": "timestamp CSV has no frames",
        })
        return

    span = (last_ns - first_ns) / 1e9 if (first_ns is not None and last_ns) else 0.0
    problems = []
    if duration > 0 and span < duration * MIN_SPAN_FRACTION:
        problems.append(
            f"video spans {span:.0f}s of a {duration:.0f}s session (recorder died mid-take?)"
        )
    if expected_hz is not None and span > 0:
        hz = frames / span
        min_hz = expected_hz * min_rate_factor
        if hz < min_hz:
            problems.append(f"frame rate {hz:.2f} Hz below minimum {min_hz:.2f} Hz")

    checks.append({
        "topic": topic, "sink": "video",
        "status": "fail" if problems else "ok",
        "count": frames, "span_s": round(span, 1),
        "hz": round(frames / span, 2) if span > 0 else None,
        "expected_hz": expected_hz,
        "detail": "; ".join(problems),
    })


def _check_transport_loss(checks: list, session_dir: str):
    log_path = os.path.join(session_dir, "rosbag_record.log")
    if not os.path.isfile(log_path):
        return
    lost = 0
    pattern = re.compile(r"messages lost on the transport layer:\s*(\d+)")
    with open(log_path, "r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            match = pattern.search(line)
            if match:
                lost += int(match.group(1))
    if lost:
        checks.append({
            "topic": "rosbag transport", "sink": "bag", "status": "warn",
            "count": lost, "detail": f"{lost} messages lost on the transport layer",
        })


def run_check(session_dir: str) -> int:
    manifest_path = os.path.join(session_dir, "expected_topics.yaml")
    if not os.path.isfile(manifest_path):
        print(f"FAIL: no manifest at {manifest_path}")
        return 1
    manifest = _load_yaml(manifest_path)
    min_rate_factor = float(
        (manifest.get("session", {}) or {}).get("min_rate_factor", 0.5)
    )

    metadata, metadata_path = _bag_metadata(session_dir)
    counts = _bag_topic_counts(metadata)
    duration = _bag_duration_s(metadata)
    videos_dir = os.path.join(session_dir, "videos")

    checks: list[dict] = []
    expected = manifest.get("expected_topics", []) or []
    bag_expected = [e for e in expected if e.get("sink") == "bag"]
    video_expected = [e for e in expected if e.get("sink") == "video"]

    if bag_expected and not metadata:
        checks.append({
            "topic": "<bag>", "sink": "bag", "status": "fail",
            "detail": "no bag metadata.yaml found",
        })

    if metadata:
        for entry in bag_expected:
            topic = str(entry["topic"])
            _check_rate(
                checks, topic, counts.get(topic, 0), duration,
                entry.get("expected_hz"), min_rate_factor, "bag",
            )
        exclude = manifest.get("bag_exclude_regex") or ""
        for entry in manifest.get("expected_regex", []) or []:
            regex = re.compile(str(entry["regex"]))
            matched = [
                t for t in counts
                if regex.fullmatch(t) and not (exclude and re.fullmatch(exclude, t))
            ]
            if not matched:
                checks.append({
                    "topic": entry["regex"], "sink": "bag", "status": "fail",
                    "detail": "regex matched no recorded topics",
                })
            for topic in sorted(matched):
                _check_rate(
                    checks, topic, counts[topic], duration,
                    entry.get("expected_hz"), min_rate_factor, "bag",
                )

    for entry in video_expected:
        _check_video(
            checks, str(entry["topic"]), videos_dir, duration,
            entry.get("expected_hz"), min_rate_factor,
        )

    _check_transport_loss(checks, session_dir)

    failed = [c for c in checks if c["status"] == "fail"]
    warned = [c for c in checks if c["status"] == "warn"]
    status = "FAIL" if failed else "PASS"

    report = {
        "session_dir": session_dir,
        "checked_at": datetime.now().isoformat(timespec="seconds"),
        "bag_metadata": metadata_path,
        "bag_duration_s": round(duration, 1),
        "status": status,
        "failed": len(failed),
        "warnings": len(warned),
        "checks": checks,
    }
    report_path = os.path.join(session_dir, "session_report.yaml")
    with open(report_path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(report, handle, sort_keys=False)

    for check in failed + warned:
        print(f"[{check['status'].upper()}] {check['topic']}: {check['detail']}")
    print(
        f"{status}: {len(checks) - len(failed)}/{len(checks)} checks ok, "
        f"{len(warned)} warnings ({report_path})"
    )
    return 1 if failed else 0


def main(args=None):
    argv = args if args is not None else sys.argv[1:]
    if len(argv) != 1:
        print("usage: session_check <session_dir>")
        return 2
    return run_check(argv[0])


if __name__ == "__main__":
    sys.exit(main())
