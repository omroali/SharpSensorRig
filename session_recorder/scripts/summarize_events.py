#!/usr/bin/env python3
"""Summarise JSON events recorded on a bag topic (default /sharp/events).

The rig records an extra topic (``bag_extra_topics`` in recording.yaml) that
carries annotation events as JSON in a ``std_msgs/String``. This reads that
topic straight from the session bag and prints:

  * how many messages it holds and the ROS message type
  * the union of JSON keys with their value types
  * distinct values of low-cardinality fields (the likely activity/event labels)
  * a few full records
  * optionally, every record as a JSON array

It is the fastest way to learn the event schema without a live publisher.

Usage:
  python3 summarize_events.py <session_dir_or_bag_dir> [--topic /sharp/events]
  python3 summarize_events.py <session> --out /tmp/events.json
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from collections import Counter

import yaml


def find_bag(path: str) -> str:
    """Accept a session dir (contains bag*/) or a bag dir (contains metadata.yaml)."""
    if os.path.isfile(os.path.join(path, "metadata.yaml")):
        return path
    candidates = sorted(glob.glob(os.path.join(path, "bag*", "metadata.yaml")))
    if not candidates:
        sys.exit(f"ERROR: no bag*/metadata.yaml under {path}")
    return os.path.dirname(candidates[-1])


def load_info(bag_dir: str) -> dict:
    with open(os.path.join(bag_dir, "metadata.yaml"), "r", encoding="utf-8") as handle:
        return (yaml.safe_load(handle) or {}).get("rosbag2_bagfile_information", {}) or {}


def topic_types(info: dict) -> dict[str, str]:
    out: dict[str, str] = {}
    for entry in info.get("topics_with_message_count", []) or []:
        metadata = entry.get("topic_metadata", {}) or {}
        if metadata.get("name"):
            out[str(metadata["name"])] = str(metadata.get("type", ""))
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", help="Session directory or bag directory")
    parser.add_argument("--topic", default="/sharp/events")
    parser.add_argument("--max-print", type=int, default=5)
    parser.add_argument("--out", default=None, help="Write all events as a JSON array")
    args = parser.parse_args(argv)

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag_dir = find_bag(args.path)
    info = load_info(bag_dir)
    types = topic_types(info)

    if args.topic not in types:
        print(f"'{args.topic}' is not in {bag_dir}.")
        related = [t for t in types if "event" in t.lower() or "sharp" in t.lower()]
        if related:
            print("Topics that might be it:")
            for topic in related:
                print(f"  {topic}  ({types[topic]})")
        else:
            print(f"Bag has {len(types)} topics; none look event-like.")
        return 1

    type_str = types[args.topic]
    msg_type = get_message(type_str)
    storage_id = str(info.get("storage_identifier", "mcap"))

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    try:
        reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))
    except Exception:
        pass  # older API; fall back to filtering in the loop below

    events = []
    keys: Counter = Counter()
    value_types: dict[str, str] = {}

    while reader.has_next():
        topic, data, _stamp = reader.read_next()
        if topic != args.topic:
            continue
        msg = deserialize_message(data, msg_type)
        raw = getattr(msg, "data", None)
        try:
            obj = json.loads(raw) if isinstance(raw, str) else raw
        except (TypeError, ValueError):
            obj = {"_raw": raw}
        events.append(obj)
        if isinstance(obj, dict):
            for key, value in obj.items():
                keys[key] += 1
                value_types.setdefault(key, type(value).__name__)

    print(f"{args.topic}: {len(events)} messages  ({type_str})")
    if not events:
        print("No messages on this topic — no live activity annotations to use.")
        return 0

    print("\nJSON keys:")
    for key, count in keys.most_common():
        print(f"  {key}: {value_types[key]}  (in {count}/{len(events)})")

    print("\nCandidate label fields (distinct values):")
    for key in keys:
        values = [str(e.get(key)) for e in events if isinstance(e, dict) and key in e]
        uniq = sorted(set(values))
        if 1 <= len(uniq) <= 25:
            print(f"  {key}: {uniq}")

    print(f"\nFirst {min(args.max_print, len(events))} records:")
    for event in events[: args.max_print]:
        print("  " + json.dumps(event, default=str))

    if args.out:
        with open(args.out, "w", encoding="utf-8") as handle:
            json.dump(events, handle, indent=2, default=str)
        print(f"\nWrote {args.out}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
