#!/usr/bin/env python3
"""Replay H.265 videos into ROS Image topics using timestamp CSVs."""

import csv
import os
import threading

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header


def load_timestamps(csv_path: str):
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"Timestamp CSV not found: {csv_path}")

    rows = []
    with open(csv_path, newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append((int(row["frame_idx"]), int(row["ros_timestamp_ns"])))

    rows.sort(key=lambda r: r[0])
    return rows


def make_default_camera_info(width: int, height: int) -> CameraInfo:
    info = CameraInfo()
    info.width = width
    info.height = height
    info.distortion_model = "plumb_bob"
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

    fx = float(max(width, height))
    fy = fx
    cx = float(width - 1) / 2.0
    cy = float(height - 1) / 2.0

    info.k = [
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0,
    ]
    info.r = [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    ]
    info.p = [
        fx, 0.0, cx, 0.0,
        0.0, fy, cy, 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    return info


def camera_info_topic(image_topic: str) -> str:
    if image_topic.endswith("/image"):
        return image_topic[:-6] + "camera_info"
    return image_topic + "/camera_info"


class VideoStream:
    def __init__(self, video_path, csv_path, topic, encoding, queue_size, frame_id, node: Node):
        self.topic = topic
        self.encoding = encoding
        self.frame_id = frame_id
        self._node = node

        self._timestamps = load_timestamps(csv_path)
        self._cursor = 0
        self._total = len(self._timestamps)

        self._cap = cv2.VideoCapture(video_path)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open video: {video_path}")

        self._pub = node.create_publisher(Image, topic, queue_size)
        self._info_pub = node.create_publisher(CameraInfo, camera_info_topic(topic), queue_size)
        self._camera_info = make_default_camera_info(
            int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 0,
            int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 0,
        )
        self._lock = threading.Lock()

        node.get_logger().info(
            f"VideoStream ready: {os.path.basename(video_path)}\n"
            f"  -> {topic} ({self._total} frames)"
        )

    @property
    def finished(self) -> bool:
        return self._cursor >= self._total

    @property
    def next_ts_ns(self):
        if self.finished:
            return None
        return self._timestamps[self._cursor][1]

    def publish_if_due(self, clock_ns: int) -> bool:
        with self._lock:
            if self.finished:
                return False

            frame_idx, ts_ns = self._timestamps[self._cursor]
            if clock_ns < ts_ns:
                return False

            current_pos = int(self._cap.get(cv2.CAP_PROP_POS_FRAMES))
            if current_pos != frame_idx:
                self._cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)

            ok, frame = self._cap.read()
            if not ok:
                self._node.get_logger().warn(
                    f"[{self.topic}] Failed to read frame {frame_idx} - skipping."
                )
                self._cursor += 1
                return False

            msg = Image()
            msg.header = Header()
            msg.header.stamp.sec = int(ts_ns // 1_000_000_000)
            msg.header.stamp.nanosec = int(ts_ns % 1_000_000_000)
            msg.header.frame_id = self.frame_id
            msg.height, msg.width = frame.shape[:2]
            msg.encoding = self.encoding
            msg.is_bigendian = False
            msg.step = int(frame.strides[0])

            if self.encoding.lower() == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                msg.data = frame.tobytes()
            elif self.encoding.lower() == "mono8":
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                msg.data = gray.tobytes()
            else:
                msg.data = frame.tobytes()

            self._pub.publish(msg)
            self._camera_info.header = msg.header
            self._info_pub.publish(self._camera_info)

            self._cursor += 1
            return True


class VideoToImagePublisher(Node):
    def __init__(self):
        super().__init__("video_to_image_publisher")

        # A type-only declaration (``Parameter.Type.STRING_ARRAY``) leaves the
        # parameter uninitialized in Jazzy, and reading an uninitialized
        # parameter raises ParameterUninitializedException. Give each array a
        # concrete default; empty entries are filtered out below.
        self.declare_parameter("videos", [""])
        self.declare_parameter("timestamp_csvs", [""])
        self.declare_parameter("topics", [""])
        self.declare_parameter("frame_ids", [""])
        self.declare_parameter("queue_size", 10)
        self.declare_parameter("encoding", "bgr8")
        # `use_sim_time` is a built-in parameter that rclpy declares on every
        # node (Jazzy and later), so declaring it unconditionally raises
        # ParameterAlreadyDeclaredException. Only declare it if absent, which
        # keeps this working on older distros too.
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)

        videos = [str(v) for v in self.get_parameter("videos").value if v]
        csvs = [str(v) for v in self.get_parameter("timestamp_csvs").value if v]
        topics = [str(v) for v in self.get_parameter("topics").value if v]
        frame_ids = [str(v) for v in self.get_parameter("frame_ids").value if v]
        queue_size = int(self.get_parameter("queue_size").value)
        encoding = str(self.get_parameter("encoding").value)

        if len(videos) != len(csvs) or len(videos) != len(topics):
            raise RuntimeError("videos, timestamp_csvs, and topics must match in length")

        if frame_ids and len(frame_ids) != len(videos):
            raise RuntimeError("frame_ids length must match videos length")

        if not frame_ids:
            frame_ids = [""] * len(videos)

        self._streams = [
            VideoStream(video, csv_path, topic, encoding, queue_size, frame_id, self)
            for video, csv_path, topic, frame_id in zip(videos, csvs, topics, frame_ids)
        ]

        # `/clock` is conventionally published BEST_EFFORT (rosbag2_player
        # --clock, rclcpp's ClockQoS). A default RELIABLE subscription is
        # incompatible with that and silently receives nothing, which stalls
        # the whole node because publishing is clock-driven. BEST_EFFORT also
        # matches a RELIABLE publisher, so this works either way.
        clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._clock_sub = self.create_subscription(Clock, "/clock", self._on_clock, clock_qos)
        self._latest_clock = 0

    def _on_clock(self, msg: Clock):
        self._latest_clock = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        for stream in self._streams:
            stream.publish_if_due(self._latest_clock)


def main(args=None):
    rclpy.init(args=args)
    node = VideoToImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
