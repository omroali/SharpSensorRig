#!/usr/bin/env python3
"""Scrubbable session player for reviewing SHARP recordings.

Unlike ``ros2 bag play``, this node can *seek*: the bag is opened with
``rosbag2_py`` (random access) and the colour videos are seeked in step via
their timestamp CSVs, so you can scrub backwards and forwards, step through
time, and jump between activities (from ``<session>/activities.yaml``, produced
by ``session_activities``).

It publishes only what RViz displays — colour frames, depth images with their
matching ``camera_info`` (so ``depth_image_proc`` can rebuild clouds), and
``/tf_static`` — restamped to the current wall clock. Because it doesn't rely on
``/clock``, RViz runs in normal (wall) time and static TF is timeless, which
avoids the whole sim-time QoS problem.

Controls (terminal):
  space        play / pause
  n / p        jump to next / previous activity   (also ] / [)
  h / l        seek -1 s / +1 s
  q            quit

Also drives the same actions over the ROS interface, for a future RViz panel:
  /session/play, /session/pause, /session/next_activity, /session/prev_activity
  (std_srvs/Trigger); /session/seek (std_msgs/Float64, seconds from session
  start) and /session/seek_fraction (std_msgs/Float64, 0..1) as topics;
  /session/status (std_msgs/String, JSON progress + current activity).

Usage:
  ros2 run session_recorder session_player <session_dir> [--rate 1.0] [--no-rviz]
"""

from __future__ import annotations

import argparse
import bisect
import json
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque

import yaml

from session_recorder.activities import ActivityIndex
from session_recorder.replay_session import (
    camera_info_for,
    discover_depths,
    discover_extra_topics,
    discover_videos,
    find_bag_dir,
    load_bag_topics,
    load_expected,
    write_rviz_config,
)
from session_recorder.tf_overrides import (
    describe_overrides,
    load_overrides,
    merge_overrides,
    parse_override_spec,
)
from session_recorder.tf_static_replay import publish_with_overrides

EVENTS_TOPIC = "/sharp/events"
GUARD_NS = 1_000_000_000  # seek this far before the target, then read forward
LOOKAHEAD_NS = 500_000_000  # how far ahead of the clock to buffer the bag
MAX_PER_TOPIC = 400  # cap queued messages per topic (memory bound)
KEY_STEP_NS = 1_000_000_000


# ── pure helpers (unit-tested) ──────────────────────────────────────────────


def frame_index_at(timestamps: list[int], ns: int) -> int:
    """Index of the last frame at or before ``ns``; -1 if none."""
    return bisect.bisect_right(timestamps, ns) - 1


def activity_jump_target(activities: list[dict], current_ns: int, direction: int):
    """Start time of the next/previous activity relative to ``current_ns``.

    ``direction > 0`` -> smallest start strictly after now (or None).
    ``direction < 0`` -> largest start strictly before now, else 0.0 (start).
    """
    starts = sorted(int(a["start_ns"]) for a in activities if a.get("start_ns") is not None)
    if not starts:
        return None
    if direction > 0:
        pos = bisect.bisect_right(starts, current_ns)
        return starts[pos] if pos < len(starts) else None
    pos = bisect.bisect_left(starts, current_ns) - 1
    return starts[pos] if pos >= 0 else 0


def progress_fraction(start_ns: int, end_ns: int, ns: int) -> float:
    if end_ns <= start_ns:
        return 0.0
    return min(1.0, max(0.0, (ns - start_ns) / (end_ns - start_ns)))


def bag_start_ns(info: dict) -> int | None:
    """Bag start time in ns, or None when the metadata doesn't state one.

    rosbag2 metadata v9 (Jazzy) stores ``starting_time.nanoseconds_since_epoch``;
    older bags used ``starting_time.nanoseconds``. Reading only the old key
    silently yields 0, which puts the session clock ~56 years before every bag,
    video and event timestamp — scrubbing then looks dead and activities never
    match.
    """
    starting = info.get("starting_time") or {}
    for key in ("nanoseconds_since_epoch", "nanoseconds"):
        value = starting.get(key)
        if value is not None:
            return int(value)
    return None


def _mmss(ns: int) -> str:
    seconds = max(0, int(ns) // 1_000_000_000)
    return f"{seconds // 60:02d}:{seconds % 60:02d}"


def human_status(st: dict) -> str:
    """One-line status used both in the terminal and the RViz text HUD."""
    state = "PLAY " if st.get("playing") else "PAUSE"
    pos = _mmss(st.get("current_ns", 0) - st.get("start_ns", 0))
    total = _mmss(st.get("end_ns", 0) - st.get("start_ns", 0))
    pct = round(100 * st.get("fraction", 0.0))
    if st.get("action_id"):
        activity = f"{st['action_id']} {st.get('action_name') or ''}".strip()
        tier = st.get("tier") or ""
        activity = f"{activity} ({tier})  #{st.get('activity_index')}"
    else:
        activity = "no activity"
    return f"[{state}] {pos}/{total} {pct:3d}%  {activity}"


# ── node ────────────────────────────────────────────────────────────────────


class SessionPlayer:
    """Wraps the rclpy node; split out so imports stay test-friendly."""

    def __init__(self, session_dir, bag_dir, videos, depths, activities, rate, logger,
                 extra_topics=None):
        self.session_dir = session_dir
        self.bag_dir = bag_dir
        self.videos = videos
        self.depths = depths
        self.activities = activities
        self.rate = rate
        self.log = logger

        self._build_time_range(bag_dir, videos)
        self._current_ns = self.start_ns
        self.playing = True
        self.running = True
        # OpenCV's H.265 decoder is not safe under concurrent use, and the
        # keyboard thread and the tick timer both drive it -> serialise.
        self._lock = threading.RLock()

        self._depths_by_image = {d["image"]: d for d in depths}
        self._act_index = ActivityIndex(self.activities)
        self._pubs: dict = {}         # topic -> (publisher, message_type)
        self._video_pubs: dict = {}   # publish_topic -> publisher
        self._aliases: dict = {}      # topic -> [extra publishers]
        self._display_topics = sorted(
            {d["image"] for d in depths}
            | {d["camera_info"] for d in depths}
            # Colour camera_info is replayed too, so each video stream can adopt
            # its recorded frame_id (an empty frame draws the image at the fixed
            # frame instead of at the camera).
            | {t for t in (camera_info_for(v.get("record_topic") or "") for v in videos) if t}
            | set(extra_topics or ())
        )
        self._queues: dict[str, deque] = {}
        self._reader = None
        self._reader_done = False
        self._open_streams(videos)

    # -- time range --

    def _build_time_range(self, bag_dir, videos):
        meta_path = os.path.join(bag_dir, "metadata.yaml")
        with open(meta_path, "r", encoding="utf-8") as handle:
            info = (yaml.safe_load(handle) or {}).get("rosbag2_bagfile_information", {}) or {}
        start = bag_start_ns(info)
        duration = int((info.get("duration", {}) or {}).get("nanoseconds", 0))
        # Union the bag range with the video timestamp ranges. An absent start
        # must not contribute a 0 here, or it drags start_ns back to the Unix
        # epoch and every seek/activity lookup misses by ~56 years.
        starts: list[int] = []
        ends: list[int] = []
        if start is not None:
            starts.append(start)
            ends.append(start + duration)
        for spec in videos:
            stamps = _csv_timestamps(spec["csv"])
            if stamps:
                starts.append(stamps[0])
                ends.append(stamps[-1])
        self.start_ns = min(starts) if starts else 0
        self.end_ns = max(ends) if ends else 0

    def _open_streams(self, videos):
        import cv2

        self._caps = []
        for spec in videos:
            stamps = _csv_timestamps(spec["csv"])
            cap = cv2.VideoCapture(spec["video"])
            self._caps.append({
                "spec": spec,
                "cap": cap,
                "timestamps": stamps,
                "last_published": -1,
                # Filled in from the recorded camera_info once it is seen; an
                # empty frame_id makes RViz draw the image at the fixed frame.
                "frame_id": "",
            })

    def seek(self, ns: int) -> None:
        with self._lock:
            self._current_ns = min(self.end_ns, max(self.start_ns, int(ns)))
            self._reader_done = False
            self._queues = {}
            self._open_reader(self._current_ns)
            self._pump_until_covered(self._current_ns)
            self._publish_snapshot()
            self._publish_videos_at(self._current_ns)

    # -- bag reading --

    def _open_reader(self, at_ns: int):
        import rosbag2_py

        with open(os.path.join(self.bag_dir, "metadata.yaml"), "r", encoding="utf-8") as handle:
            info = (yaml.safe_load(handle) or {}).get("rosbag2_bagfile_information", {}) or {}
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(
                uri=self.bag_dir, storage_id=str(info.get("storage_identifier", "mcap"))
            ),
            rosbag2_py.ConverterOptions("cdr", "cdr"),
        )
        try:
            reader.set_filter(rosbag2_py.StorageFilter(topics=list(self._display_topics)))
        except Exception:
            pass
        try:
            reader.seek(max(0, at_ns - GUARD_NS))
        except Exception:
            pass
        self._reader = reader

    def _pump_ahead(self):
        """Read ahead by a fixed time window, not a message count.

        High-rate topics (vicon at 120 Hz) would otherwise flood a fixed-size
        buffer and starve the images. Capping per-topic history keeps memory
        bounded too.
        """
        if self._reader is None or self._reader_done:
            return
        horizon = self._current_ns + LOOKAHEAD_NS
        while self._reader.has_next():
            topic, data, stamp = self._reader.read_next()
            stamp = int(stamp)
            if topic in self._display_topics:
                queue = self._queues.setdefault(topic, deque())
                queue.append((stamp, data))
                if len(queue) > MAX_PER_TOPIC:
                    queue.popleft()
            if stamp > horizon:
                break
        if not self._reader.has_next():
            self._reader_done = True

    def _pump_until_covered(self, ns: int, cap: int = 6000):
        """After a seek, read until every topic has a message past ``ns``.

        Bounded, so a seek buffers roughly one frame window rather than the
        whole bag (depth images are large).
        """
        if self._reader is None or self._reader_done:
            return
        needed = set(self._display_topics)
        read = 0
        while needed and read < cap and self._reader.has_next():
            topic, data, stamp = self._reader.read_next()
            read += 1
            if topic not in self._display_topics:
                continue
            stamp = int(stamp)
            self._queues.setdefault(topic, deque()).append((stamp, data))
            if stamp > ns:
                needed.discard(topic)
        if not self._reader.has_next():
            self._reader_done = True

    def _drain_until(self, ns: int, publish: bool):
        """Consume queued messages with stamp <= ns; optionally publish each.

        One wall stamp is used for the whole batch so depth and its
        camera_info come out with identical stamps and stay synchronisable.
        """
        stamp = self._now_ns()
        for topic, queue in self._queues.items():
            while queue and queue[0][0] <= ns:
                _old, data = queue.popleft()
                if publish and topic != "/tf_static":
                    self._publish_bag_message(topic, data, force_stamp=stamp)

    def _publish_snapshot(self):
        """Publish the most recent queued message per topic (state at now)."""
        stamp = self._now_ns()
        for topic, queue in self._queues.items():
            if topic == "/tf_static":
                continue
            selected = None
            while queue and queue[0][0] <= self._current_ns:
                selected = queue.popleft()
            if selected is None and queue:
                selected = queue[0]  # nothing ancient enough; show the next one
                queue.popleft()
            if selected is not None:
                self._publish_bag_message(topic, selected[1], force_stamp=stamp)

    # -- publishing --

    def _now_ns(self):
        return time.time_ns()

    def _publish_bag_message(self, topic, data, force_stamp=None):
        from rclpy.serialization import deserialize_message

        pub, msg_type = self._pubs[topic]
        msg = deserialize_message(data, msg_type)
        if force_stamp is not None:
            if hasattr(msg, "header"):
                msg.header.stamp.sec = force_stamp // 1_000_000_000
                msg.header.stamp.nanosec = force_stamp % 1_000_000_000
            # TFMessage carries its stamps per transform, not in a header.
            for transform in getattr(msg, "transforms", []):
                transform.header.stamp.sec = force_stamp // 1_000_000_000
                transform.header.stamp.nanosec = force_stamp % 1_000_000_000
        pub.publish(msg)
        # depth_image_proc derives camera_info from the image topic
        # (dirname(image)/camera_info), which the /throttled suffix breaks; we
        # mirror camera_info onto the derived name so the pair synchronises.
        for alias_pub in self._aliases.get(topic, ()):
            alias_pub.publish(msg)

    def _publish_videos_at(self, ns: int):
        import cv2
        from sensor_msgs.msg import Image

        stamp = self._now_ns()
        for stream in self._caps:
            timestamps = stream["timestamps"]
            if not timestamps:
                continue
            idx = frame_index_at(timestamps, ns)
            if idx < 0:
                continue
            cap = stream["cap"]
            if int(cap.get(cv2.CAP_PROP_POS_FRAMES)) != idx:
                cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
            ok, frame = cap.read()
            if not ok:
                continue
            msg = Image()
            msg.header.stamp.sec = stamp // 1_000_000_000
            msg.header.stamp.nanosec = stamp % 1_000_000_000
            msg.header.frame_id = stream.get("frame_id", "")
            msg.height, msg.width = frame.shape[:2]
            msg.encoding = "bgr8"
            msg.is_bigendian = False
            msg.step = int(frame.strides[0])
            msg.data = frame.tobytes()
            stream["last_published"] = idx
            self._video_pubs[stream["spec"]["publish_topic"]].publish(msg)

    # -- playback tick --

    def tick(self, dt_s: float):
        with self._lock:
            if self.playing:
                self._current_ns += int(dt_s * 1e9 * self.rate)
                if self._current_ns >= self.end_ns:
                    self._current_ns = self.end_ns
                    self.playing = False
            self._pump_ahead()
            self._drain_until(self._current_ns, publish=True)
            self._publish_videos_step(self._current_ns)

    def _publish_videos_step(self, ns: int):
        import cv2
        from sensor_msgs.msg import Image

        for stream in self._caps:
            timestamps = stream["timestamps"]
            if not timestamps:
                continue
            idx = frame_index_at(timestamps, ns)
            if idx < 0 or idx == stream["last_published"]:
                continue
            cap = stream["cap"]
            if int(cap.get(cv2.CAP_PROP_POS_FRAMES)) != idx:
                cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
            ok, frame = cap.read()
            if not ok:
                continue
            msg = Image()
            stamp = self._now_ns()
            msg.header.stamp.sec = stamp // 1_000_000_000
            msg.header.stamp.nanosec = stamp % 1_000_000_000
            msg.height, msg.width = frame.shape[:2]
            msg.encoding = "bgr8"
            msg.is_bigendian = False
            msg.step = int(frame.strides[0])
            msg.data = frame.tobytes()
            stream["last_published"] = idx
            self._video_pubs[stream["spec"]["publish_topic"]].publish(msg)

    def status_dict(self) -> dict:
        activity = self._act_index.activity_at(self._current_ns)
        return {
            "current_ns": self._current_ns,
            "start_ns": self.start_ns,
            "end_ns": self.end_ns,
            "fraction": round(progress_fraction(self.start_ns, self.end_ns, self._current_ns), 4),
            "playing": self.playing,
            "activity_index": None if activity is None else activity["index"],
            "action_id": None if activity is None else activity.get("action_id"),
            "action_name": None if activity is None else activity.get("action_name"),
            "tier": None if activity is None else activity.get("tier"),
            "activity_count": len(self.activities),
        }

    def status(self) -> str:
        return json.dumps(self.status_dict())

    def jump(self, direction: int):
        target = activity_jump_target(self.activities, self._current_ns, direction)
        if target is None:
            self.log.info("No further activity in that direction")
            return
        self.seek(int(target))


def _csv_timestamps(path: str) -> list[int]:
    stamps: list[int] = []
    try:
        import csv as _csv

        with open(path, newline="", encoding="utf-8") as handle:
            for row in _csv.DictReader(handle):
                try:
                    stamps.append(int(row["ros_timestamp_ns"]))
                except (KeyError, TypeError, ValueError):
                    continue
    except OSError:
        return []
    stamps.sort()
    return stamps


# ── ROS wiring ──────────────────────────────────────────────────────────────


def _publish_tf_static(node, bag_dir, info, types, transient_qos, overrides=None):
    """Publish every ``/tf_static`` message from the bag once, latched.

    Overridden child frames are dropped from the recorded messages and replaced
    with the override, so each frame keeps exactly one parent.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    topic = "/tf_static"
    if topic not in types:
        return
    msg_type = get_message(types[topic])
    pub = node.create_publisher(msg_type, topic, transient_qos)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=bag_dir, storage_id=str(info.get("storage_identifier", "mcap"))
        ),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    try:
        reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    except Exception:
        pass

    messages = []
    while reader.has_next():
        name, data, _stamp = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, msg_type)
        for transform in getattr(msg, "transforms", []):
            transform.header.stamp.sec = 0
            transform.header.stamp.nanosec = 0
        messages.append(msg)

    if overrides:
        applied = publish_with_overrides(pub, messages, overrides)
        node.get_logger().info(
            f"Published {len(messages)} /tf_static message(s) with overrides:\n"
            f"{describe_overrides(applied)}"
        )
        return

    for msg in messages:
        pub.publish(msg)
    node.get_logger().info(f"Published {len(messages)} /tf_static message(s)")


def _build_node(session_dir, bag_dir, videos, depths, rate, extra_topics=None,
                tf_overrides=None):
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import Image

    node = Node("session_player")

    with open(os.path.join(bag_dir, "metadata.yaml"), "r", encoding="utf-8") as handle:
        info = (yaml.safe_load(handle) or {}).get("rosbag2_bagfile_information", {}) or {}
    types = {}
    for entry in info.get("topics_with_message_count", []) or []:
        metadata = entry.get("topic_metadata", {}) or {}
        if metadata.get("name"):
            types[str(metadata["name"])] = str(metadata.get("type", ""))

    sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    transient_qos = QoSProfile(
        depth=50, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    player = SessionPlayer(
        session_dir=session_dir, bag_dir=bag_dir, videos=videos, depths=depths,
        activities=_load_activities(session_dir), rate=rate, logger=node.get_logger(),
        extra_topics=extra_topics,
    )

    # Publishers for every bag topic we replay, plus the colour topics.
    player._pubs = {}
    for topic in list(player._display_topics):
        if topic not in types:
            continue
        qos = transient_qos if topic == "/tf_static" else sensor_qos
        player._pubs[topic] = (
            node.create_publisher(get_message(types[topic]), topic, qos),
            get_message(types[topic]),
        )
    player._video_pubs = {
        spec["publish_topic"]: node.create_publisher(Image, spec["publish_topic"], sensor_qos)
        for spec in videos
    }

    # depth_image_proc derives its camera_info topic from the image topic
    # (dirname(image)/camera_info). The /throttled suffix on the RealSense depth
    # topics makes that resolve to ".../image_rect_raw/camera_info" instead of
    # the recorded ".../depth/camera_info", so mirror camera_info onto the
    # derived name too. Kinect's raw sd image already derives to the right one.
    player._aliases = {}
    for spec in depths:
        derived = os.path.dirname(spec["image"]) + "/camera_info"
        camera_info = spec["camera_info"]
        if derived == camera_info or camera_info not in types:
            continue
        alias_pub = node.create_publisher(get_message(types[camera_info]), derived, sensor_qos)
        player._aliases.setdefault(camera_info, []).append(alias_pub)
        node.get_logger().info(f"Mirroring {camera_info} -> {derived}")

    # /tf_static is latched and lives at the very start of the bag, so replay it
    # once up front with transient_local durability rather than hunting for it
    # after every seek. RViz (and tf2) then has the whole frame tree.
    _publish_tf_static(node, bag_dir, info, types, transient_qos, overrides=tf_overrides)

    # Services + status (for a future RViz panel / remote control).
    from std_msgs.msg import Float64, String
    from std_srvs.srv import Trigger

    node.create_service(Trigger, "/session/play", lambda req, res: _set_play(player, res, True))
    node.create_service(Trigger, "/session/pause", lambda req, res: _set_play(player, res, False))
    node.create_service(Trigger, "/session/next_activity", lambda req, res: _jump(player, res, +1))
    node.create_service(Trigger, "/session/prev_activity", lambda req, res: _jump(player, res, -1))
    # Seek is a topic, not a service: std_msgs has no float service type, and a
    # UI can publish a Float64 directly instead of us defining a custom .srv.
    node.create_subscription(Float64, "/session/seek", lambda msg: _seek_seconds(player, msg.data), 10)
    node.create_subscription(
        Float64, "/session/seek_fraction", lambda msg: _seek_fraction(player, msg.data), 10
    )
    status_pub = node.create_publisher(String, "/session/status", 10)
    status_text_pub = node.create_publisher(String, "/session/status_text", 10)

    # Colour images adopt the frame_id of the recorded camera_info for their
    # stream (which the player republishes), so RViz places them at the camera.
    # BEST_EFFORT subscription: the player publishes camera_info with
    # sensor-data QoS, which a RELIABLE subscriber cannot receive.
    for stream in player._caps:
        info_topic = camera_info_for(stream["spec"].get("record_topic") or "")
        if not info_topic or info_topic not in types:
            continue
        node.create_subscription(
            get_message(types[info_topic]), info_topic,
            lambda msg, s=stream, t=info_topic: _adopt_video_frame(node, s, t, msg),
            sensor_qos,
        )

    def _tick():
        player.tick(node._player_dt)
        st = player.status_dict()
        msg = String()
        msg.data = json.dumps(st)
        status_pub.publish(msg)
        text = String()
        text.data = human_status(st)
        status_text_pub.publish(text)

    node._player_dt = 1.0 / 30.0
    node.create_timer(node._player_dt, _tick)

    return node, player


def _set_play(player, response, playing):
    player.playing = playing
    response.success = True
    response.message = "playing" if playing else "paused"
    return response


def _jump(player, response, direction):
    player.jump(direction)
    response.success = True
    response.message = player.status()
    return response


def _adopt_video_frame(node, stream, info_topic, msg):
    """Use the recorded camera_info frame for a colour stream, once."""
    frame_id = str(getattr(msg.header, "frame_id", ""))
    if not frame_id or stream.get("frame_id"):
        return
    stream["frame_id"] = frame_id
    node.get_logger().info(
        f"[{stream['spec']['publish_topic']}] frame_id <- {frame_id} (from {info_topic})"
    )


def _seek_seconds(player, seconds):
    player.seek(player.start_ns + int(seconds * 1e9))


def _seek_fraction(player, fraction):
    span = max(0, player.end_ns - player.start_ns)
    player.seek(player.start_ns + int(fraction * span))


def _load_activities(session_dir):
    path = os.path.join(session_dir, "activities.yaml")
    if not os.path.isfile(path):
        return []
    with open(path, "r", encoding="utf-8") as handle:
        return (yaml.safe_load(handle) or {}).get("activities", []) or []


# ── terminal controls ───────────────────────────────────────────────────────


def _keyboard_loop(player, node):
    import termios
    import tty

    import rclpy

    if not sys.stdin.isatty():
        return
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok() and player.running:
            ch = sys.stdin.read(1)
            if not ch:
                break
            if ch == " ":
                player.playing = not player.playing
            elif ch == "q":
                player.running = False
                break
            elif ch == "n" or ch == "]":
                player.jump(+1)
            elif ch == "p" or ch == "[":
                player.jump(-1)
            elif ch == "h":
                player.seek(player._current_ns - KEY_STEP_NS)
            elif ch == "l":
                player.seek(player._current_ns + KEY_STEP_NS)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ── main ────────────────────────────────────────────────────────────────────


def _find_activities_prompt(session_dir):
    if not os.path.isfile(os.path.join(session_dir, "activities.yaml")):
        print("NOTE: no activities.yaml — run `ros2 run session_recorder "
              "session_activities <session>` first to enable activity jumps.")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="session_player", description=__doc__)
    parser.add_argument("session_dir")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--paused", action="store_true", help="Start paused")
    parser.add_argument("--no-rviz", action="store_true")
    parser.add_argument("--no-pointclouds", action="store_true")
    parser.add_argument(
        "--tf-override", action="append", default=[], metavar="SPEC",
        help="Move a recorded frame: CHILD=[PARENT:]X,Y,Z[@ROLL,PITCH,YAW]; repeatable",
    )
    args = parser.parse_args(argv)

    session = os.path.abspath(args.session_dir)
    if not os.path.isdir(session):
        print(f"ERROR: not a directory: {session}", file=sys.stderr)
        return 2
    bag_dir = find_bag_dir(session)
    if bag_dir is None:
        print(f"ERROR: no bag*/metadata.yaml under {session}", file=sys.stderr)
        return 2

    _find_activities_prompt(session)
    tf_overrides = merge_overrides(
        load_overrides(session),
        [parse_override_spec(spec) for spec in (args.tf_override or [])],
    )
    if tf_overrides:
        print(f"TF overrides: {len(tf_overrides)} frame(s) will be moved "
              f"({', '.join(ov['child'] for ov in tf_overrides)})")
    bag_topics = load_bag_topics(bag_dir)
    videos = discover_videos(session, load_expected(session))
    depths = [] if args.no_pointclouds else discover_depths(bag_topics)
    extras = discover_extra_topics(bag_topics)
    markers = [
        topic for topic in extras
        if topic.startswith("/vicon/")
        and bag_topics.get(topic) == "geometry_msgs/msg/PointStamped"
    ]

    import rclpy

    rclpy.init(args=[])
    node, player = _build_node(
        session, bag_dir, videos, depths, args.rate, extras, tf_overrides
    )
    node.get_logger().info(
        f"Replaying {len(videos)} colour, {len(depths)} depth, "
        f"{len(extras)} extra topic(s) incl. /tf and {len(markers)} vicon marker topic(s)"
    )

    procs: list[subprocess.Popen] = []
    # Local point-cloud helper: NO use_sim_time, because the player publishes on
    # the wall clock (there is no /clock), and matching is by header stamp.
    def _pointcloud_cmd(spec):
        return [
            "ros2", "run", "depth_image_proc", "point_cloud_xyz_node", "--ros-args",
            "-r", f"camera_info:={spec['camera_info']}",
            "-r", f"image_rect:={spec['image']}",
            "-r", f"points:={spec['points']}",
        ]

    for spec in depths:
        procs.append(subprocess.Popen(_pointcloud_cmd(spec), start_new_session=True))
    if not args.no_rviz:
        cfg = os.path.join(session, "replay.rviz")
        write_rviz_config(cfg, videos, depths, markers, status_panel=True)
        procs.append(subprocess.Popen(
            ["ros2", "run", "rviz2", "rviz2", "-d", cfg], start_new_session=True,
        ))

    player.seek(player.start_ns)
    player.playing = not args.paused
    thread = threading.Thread(target=_keyboard_loop, args=(player, node), daemon=True)
    thread.start()

    node.get_logger().info(
        "Controls: space=play/pause  n/p=activity  h/l=seek  q=quit"
    )
    print(human_status(player.status_dict()), flush=True)
    last_key = None
    last_beat = 0.0
    try:
        while rclpy.ok() and player.running:
            rclpy.spin_once(node, timeout_sec=0.05)
            st = player.status_dict()
            key = (st["playing"], st["activity_index"])
            now = time.time()
            # Print on any change (activity / play-pause), plus a slow heartbeat.
            if key != last_key or now - last_beat >= 5.0:
                print(human_status(st), flush=True)
                last_key = key
                last_beat = now
    except KeyboardInterrupt:
        pass
    finally:
        for proc in procs:
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except (ProcessLookupError, PermissionError):
                    pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
