#!/usr/bin/env python3
"""Republish a session's recorded ``/tf_static`` with overrides applied.

``ros2 bag play`` replays ``/tf_static`` verbatim, so a frame that was published
at the wrong pose during recording (or a child frame that ended up with two
parents, e.g. ``map -> realsense_d55_1_link`` from the RealSense launch plus
``kinect2_1_link -> realsense_d55_1_link`` from the marker calibration) cannot be
fixed at replay time by editing the rig config.

This node reads ``/tf_static`` out of the bag once, drops every transform whose
child frame is overridden, publishes the rest latched (transient_local) plus the
overrides, and then idles so late subscribers still receive the latched tree.

It is normally started for you by ``replay_session`` whenever the session has
``tf_overrides.yaml`` (bag playback then excludes ``/tf_static``); running it by
hand is only useful when driving ``ros2 bag play`` yourself.

Usage:
  ros2 run session_recorder tf_static_replay <session_dir> [--tf-override SPEC]
  python3 -m session_recorder.tf_static_replay <session_dir> [--tf-override SPEC]

  SPEC = CHILD=[PARENT:]X,Y,Z[@ROLL,PITCH,YAW]      (degrees), repeatable
"""

from __future__ import annotations

import argparse
import os
import sys

from session_recorder.replay_session import find_bag_dir
from session_recorder.tf_overrides import (
    describe_overrides,
    load_overrides,
    merge_overrides,
    overridden_children,
    parse_override_spec,
    plan_overrides,
)

TF_STATIC_TOPIC = "/tf_static"


# ── bag access ──────────────────────────────────────────────────────────────


def load_tf_static(bag_dir: str, storage_id: str = "mcap") -> list:
    """Return the bag's ``/tf_static`` messages, deserialised, in order."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    try:
        reader.set_filter(rosbag2_py.StorageFilter(topics=[TF_STATIC_TOPIC]))
    except Exception:  # pragma: no cover - older rosbag2 without filters
        pass

    messages: list = []
    while reader.has_next():
        name, data, _stamp = reader.read_next()
        if name != TF_STATIC_TOPIC:
            continue
        messages.append(deserialize_message(data, TFMessage))
    return messages


def static_transform_qos():
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        depth=100,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def publish_with_overrides(publisher, messages: list, overrides: list) -> list[dict]:
    """Publish ``messages`` minus overridden children, plus the overrides.

    Returns the applied-override report (see :func:`tf_overrides.plan_overrides`).
    """
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage

    all_transforms = [tf for msg in messages for tf in getattr(msg, "transforms", [])]
    _kept, applied = plan_overrides(all_transforms, overrides)
    drop = overridden_children(overrides)

    if applied:
        override_msg = TFMessage()
        for entry in applied:
            stamped = TransformStamped()
            stamped.header.frame_id = entry["parent"]
            stamped.child_frame_id = entry["child"]
            stamped.header.stamp.sec = 0
            stamped.header.stamp.nanosec = 0
            x, y, z = entry["position"]
            stamped.transform.translation.x = x
            stamped.transform.translation.y = y
            stamped.transform.translation.z = z
            qx, qy, qz, qw = entry["quat"]
            stamped.transform.rotation.x = qx
            stamped.transform.rotation.y = qy
            stamped.transform.rotation.z = qz
            stamped.transform.rotation.w = qw
            override_msg.transforms.append(stamped)
        publisher.publish(override_msg)

    for msg in messages:
        transforms = [tf for tf in getattr(msg, "transforms", [])
                      if tf.child_frame_id not in drop]
        if not transforms:
            continue
        msg.transforms = transforms
        for transform in transforms:
            transform.header.stamp.sec = 0
            transform.header.stamp.nanosec = 0
        publisher.publish(msg)

    return applied


# ── main ────────────────────────────────────────────────────────────────────


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="tf_static_replay", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("session_dir", help="Session directory to read /tf_static from")
    parser.add_argument(
        "--tf-override", action="append", default=[], metavar="SPEC",
        help="CHILD=[PARENT:]X,Y,Z[@ROLL,PITCH,YAW]; repeatable",
    )
    args = parser.parse_args(argv)

    session = os.path.abspath(args.session_dir)
    bag_dir = find_bag_dir(session)
    if bag_dir is None:
        print(f"ERROR: no bag*/metadata.yaml under {session}", file=sys.stderr)
        return 2

    overrides = merge_overrides(
        load_overrides(session),
        [parse_override_spec(spec) for spec in (args.tf_override or [])],
    )
    if not overrides:
        print("tf_static_replay: no overrides configured, nothing to do")
        return 0

    import rclpy
    from tf2_msgs.msg import TFMessage

    rclpy.init(args=[])
    node = rclpy.create_node("tf_static_replay")
    publisher = node.create_publisher(TFMessage, TF_STATIC_TOPIC, static_transform_qos())

    import yaml

    storage_id = "mcap"
    metadata_path = os.path.join(bag_dir, "metadata.yaml")
    if os.path.isfile(metadata_path):
        with open(metadata_path, "r", encoding="utf-8") as handle:
            info = (yaml.safe_load(handle) or {}).get("rosbag2_bagfile_information", {}) or {}
        storage_id = str(info.get("storage_identifier", "mcap"))

    messages = load_tf_static(bag_dir, storage_id)
    applied = publish_with_overrides(publisher, messages, overrides)
    node.get_logger().info(
        f"tf_static_replay: {len(messages)} recorded message(s), "
        f"{len(applied)} override(s):\n{describe_overrides(applied)}"
    )

    try:
        # Keep the publisher alive: transient_local delivery happens on discovery,
        # which is after RViz/tf2 subscribers appear.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
