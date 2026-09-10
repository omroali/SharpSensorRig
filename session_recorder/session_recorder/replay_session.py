#!/usr/bin/env python3
"""Replay a recorded session for visualisation.

Takes a session directory written by recording_manager and brings the whole
take back up, everything locked to the bag's /clock so the streams stay time
aligned:

  * ``ros2 bag play <session>/bag* --clock``
  * ``video_to_image_publisher`` over every colour stream in ``<session>/videos``
  * a ``depth_image_proc/point_cloud_xyz_node`` for every depth topic in the
    bag, fed from that stream's recorded ``camera_info``
  * RViz (optional) with a generated config that already contains a display for
    every colour image, depth image and reconstructed point cloud

Discovery is driven by the session's own ``expected_topics.yaml`` (which maps
each video topic to its file via the same ``topic.strip('/').replace('/', '_')``
rule the recorder used) plus the bag's ``metadata.yaml`` (topic names and
message types). Nothing is hard-coded, so it works for any sensor mix, and for
both the raw-``sd`` Kinect depth and the throttled RealSense depth topics.

Usage:
  ros2 run session_recorder replay_session <session_dir> [options]

  # show what would be replayed without starting anything (no ROS needed):
  ros2 run session_recorder replay_session <session_dir> --list

Options:
  --rate FLOAT        bag playback speed multiplier (default 1.0)
  --loop              loop bag playback (colour plays once, then goes idle)
  --no-video          skip the colour video publisher
  --no-pointclouds    skip depth->pointcloud reconstruction
  --no-rviz           don't launch RViz
  --rviz-config PATH  write/use a specific RViz config path
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import signal
import subprocess
import sys
import time

import yaml

# Replay the same vicon set the recorder kept: everything under /vicon/ except
# the regenerable *visualization* MarkerArrays, plus the dynamic /tf stream
# (segment poses) so RViz's TF display has frames to show.
VICON_REGEX = r"/vicon/.*"
VICON_EXCLUDE_REGEX = r"/vicon/.*visualization.*"

# sensor_msgs/Image is the only type depth_image_proc can consume; compressed
# transports (CompressedImage / "/compressedDepth") would need a republish step
# and are reported but skipped.
IMAGE_TYPE = "sensor_msgs/msg/Image"


# ── session introspection ───────────────────────────────────────────────────


def topic_to_stem(topic: str) -> str:
    """Mirror colour_video_recorder.topic_to_filename so manifests map to files."""
    return topic.strip("/").replace("/", "_")


def _load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def find_bag_dir(session_dir: str) -> str | None:
    """Return the last ``bag*/`` directory containing a metadata.yaml."""
    candidates = sorted(glob.glob(os.path.join(session_dir, "bag*", "metadata.yaml")))
    if not candidates:
        return None
    return os.path.dirname(candidates[-1])


def load_bag_topics(bag_dir: str) -> dict[str, str]:
    """Map topic name -> message type using the bag's metadata.yaml."""
    metadata = _load_yaml(os.path.join(bag_dir, "metadata.yaml"))
    info = metadata.get("rosbag2_bagfile_information", {}) or {}
    topics: dict[str, str] = {}
    for entry in info.get("topics_with_message_count", []) or []:
        topic_metadata = entry.get("topic_metadata", {}) or {}
        name = topic_metadata.get("name")
        if name:
            topics[str(name)] = str(topic_metadata.get("type", ""))
    return topics


def load_expected(session_dir: str) -> dict:
    path = os.path.join(session_dir, "expected_topics.yaml")
    return _load_yaml(path) if os.path.isfile(path) else {}


def _strip_compressed(topic: str) -> str:
    """The recorder subscribes to <topic>/compressed for colour; replay raw."""
    for suffix in ("/compressedDepth", "/compressed"):
        if topic.endswith(suffix):
            return topic[: -len(suffix)]
    return topic


def discover_videos(session_dir: str, expected: dict) -> list[dict]:
    """Find colour streams and the topic to republish each one on.

    Prefers the session manifest (exact topic mapping). Any leftover ``*.mp4``
    without a manifest entry is still included, published on ``/<stem>``.
    """
    videos_dir = os.path.join(session_dir, "videos")
    specs: list[dict] = []
    seen: set[str] = set()

    entries = expected.get("expected_topics") or []
    for entry in entries:
        if entry.get("sink") != "video":
            continue
        topic = str(entry.get("topic", ""))
        if not topic:
            continue
        stem = topic_to_stem(topic)
        video = os.path.join(videos_dir, f"{stem}.mp4")
        csv_path = os.path.join(videos_dir, f"{stem}.csv")
        if not (os.path.isfile(video) and os.path.isfile(csv_path)):
            continue
        seen.add(stem)
        specs.append({
            "video": video,
            "csv": csv_path,
            "record_topic": topic,
            "publish_topic": _strip_compressed(topic),
        })

    # Fallback: videos with no manifest entry (e.g. manifest missing).
    for video in sorted(glob.glob(os.path.join(videos_dir, "*.mp4"))):
        stem = os.path.splitext(os.path.basename(video))[0]
        if stem in seen:
            continue
        csv_path = os.path.join(videos_dir, f"{stem}.csv")
        if not os.path.isfile(csv_path):
            continue
        specs.append({
            "video": video,
            "csv": csv_path,
            "record_topic": "",
            "publish_topic": f"/{stem}",
        })

    return specs


def _is_depth_image(topic: str, type_name: str) -> bool:
    if type_name != IMAGE_TYPE:
        return False
    base = topic.rstrip("/").split("/")[-1]
    if base.startswith("image_ir") or "infrared" in topic:
        return False
    return "depth" in base or "/depth/" in topic


def _clean_topic(topic: str) -> str:
    return topic.removesuffix("/throttled")


def discover_depths(bag_topics: dict[str, str]) -> list[dict]:
    """Pair each raw depth topic with its camera_info and an output points topic."""
    specs: list[dict] = []
    for topic, type_name in sorted(bag_topics.items()):
        if not _is_depth_image(topic, type_name):
            continue
        clean = _clean_topic(topic)
        root = clean.rsplit("/", 1)[0]
        camera_info = f"{root}/camera_info"
        if camera_info not in bag_topics:
            # Without calibration we cannot reconstruct geometry.
            continue
        specs.append({
            "image": topic,
            "camera_info": camera_info,
            "points": f"{root}/points",
        })
    return specs


def discover_extra_topics(bag_topics: dict[str, str]) -> list[str]:
    """Bag topics to replay beyond videos and depth: /tf and vicon data."""
    extras = ["/tf"] if "/tf" in bag_topics else []
    for topic in sorted(bag_topics):
        if re.fullmatch(VICON_REGEX, topic) and not re.fullmatch(VICON_EXCLUDE_REGEX, topic):
            extras.append(topic)
    return extras


# ── command construction ────────────────────────────────────────────────────


def _yaml_array(values: list[str]) -> str:
    return "[" + ", ".join(values) + "]"


def bag_play_cmd(bag_dir: str, rate: float, loop: bool) -> list[str]:
    cmd = ["ros2", "bag", "play", bag_dir, "--clock"]
    if rate != 1.0:
        cmd += ["--rate", str(rate)]
    if loop:
        cmd += ["--loop"]
    return cmd


def video_publisher_cmd(specs: list[dict]) -> list[str]:
    return [
        "ros2", "run", "session_recorder", "video_to_image_publisher", "--ros-args",
        "-p", f"videos:={_yaml_array([s['video'] for s in specs])}",
        "-p", f"timestamp_csvs:={_yaml_array([s['csv'] for s in specs])}",
        "-p", f"topics:={_yaml_array([s['publish_topic'] for s in specs])}",
    ]


def pointcloud_cmd(spec: dict) -> list[str]:
    return [
        "ros2", "run", "depth_image_proc", "point_cloud_xyz_node", "--ros-args",
        "-r", f"camera_info:={spec['camera_info']}",
        "-r", f"image_rect:={spec['image']}",
        "-r", f"points:={spec['points']}",
        "-p", "use_sim_time:=true",
    ]


def rviz_cmd(config_path: str) -> list[str]:
    return [
        "ros2", "run", "rviz2", "rviz2", "-d", config_path,
        "--ros-args", "-p", "use_sim_time:=true",
    ]


# ── RViz config generation ──────────────────────────────────────────────────


def _topic_qos(topic: str) -> dict:
    # Best Effort works against both Reliable and Best Effort publishers, which
    # matters because bag playback QoS follows whatever was recorded.
    return {
        "Depth": 5,
        "Durability Policy": "Volatile",
        "History Policy": "Keep Last",
        "Reliability Policy": "Best Effort",
        "Value": topic,
    }


def _grid_display() -> dict:
    return {
        "Alpha": 0.5,
        "Cell Size": 1,
        "Class": "rviz_default_plugins/Grid",
        "Color": "160; 160; 164",
        "Enabled": True,
        "Line Style": {"Line Width": 0.03, "Value": "Lines"},
        "Name": "Grid",
        "Normal Cell Count": 0,
        "Offset": {"X": 0, "Y": 0, "Z": 0},
        "Plane": "XY",
        "Plane Cell Count": 20,
        "Reference Frame": "<Fixed Frame>",
        "Value": True,
    }


def _tf_display() -> dict:
    return {
        "Class": "rviz_default_plugins/TF",
        "Enabled": True,
        "Frame Timeout": 15,
        "Frames": {"All Enabled": True},
        "Marker Scale": 0.3,
        "Name": "TF",
        "Show Arrows": True,
        "Show Axes": True,
        "Show Names": False,
        "Update Interval": 0,
    }


def _image_display(name: str, topic: str) -> dict:
    return {
        "Class": "rviz_default_plugins/Image",
        "Enabled": True,
        "Max Value": 1.0,
        "Median window": 5,
        "Min Value": 0.0,
        "Name": name,
        "Normalize Range": True,  # render 16-bit depth sensibly
        "Topic": _topic_qos(topic),
    }


def _cloud_display(name: str, topic: str) -> dict:
    return {
        "Alpha": 1.0,
        "Autocompute Intensity Bounds": True,
        "Axis": "Z",
        "Channel Name": "intensity",
        "Class": "rviz_default_plugins/PointCloud2",
        "Color": "255; 255; 255",
        "Color Transformer": "AxisColor",
        "Decay Time": 0.0,
        "Enabled": True,
        "Invert Rainbow": False,
        "Max Color": "255; 255; 255",
        "Min Color": "0; 0; 0",
        "Name": name,
        "Position Transformer": "XYZ",
        "Selectable": True,
        "Size (Pixels)": 3,
        "Size (m)": 0.005,
        "Style": "Flat Squares",
        "Topic": _topic_qos(topic),
        "Use Fixed Frame": True,
        "Use rainbow": True,
    }


def _point_display(name: str, topic: str) -> dict:
    """RViz PointStamped display for a geometry_msgs/PointStamped (vicon marker)."""
    return {
        "Class": "rviz_default_plugins/PointStamped",
        "Enabled": True,
        "Alpha": 1.0,
        "Color": "255; 255; 0",
        "Radius": 0.02,
        "Name": name,
        "Topic": _topic_qos(topic),
    }


def build_rviz_config(
    videos: list[dict],
    depths: list[dict],
    markers: list[str] | None = None,
    status_panel: bool = False,
) -> dict:
    """Build an rviz config with a display for every replayed stream.

    ``status_panel`` adds the session_recorder_rviz SessionStatusPanel, which
    shows the player's play/pause state and current activity as a dockable
    panel in the RViz window (driven by /session/status_text).
    """
    displays: list[dict] = [_grid_display(), _tf_display()]
    for spec in videos:
        displays.append(_image_display(f"Colour {spec['publish_topic']}", spec["publish_topic"]))
    for spec in depths:
        displays.append(_image_display(f"Depth {spec['image']}", spec["image"]))
        displays.append(_cloud_display(f"Points {spec['points']}", spec["points"]))
    for topic in markers or []:
        displays.append(_point_display(f"Marker {topic}", topic))

    panels: list[dict] = [
        {
            "Class": "rviz_common/Displays",
            "Name": "Displays",
            "Property Tree Widget": {
                "Expanded": ["/Global Options1"],
                "Splitter Ratio": 0.5,
            },
            "Tree Height": 600,
        },
        {"Class": "rviz_common/Views", "Name": "Views"},
    ]
    if status_panel:
        panels.append({
            "Class": "session_recorder_rviz/SessionStatusPanel",
            "Name": "Session Status",
            "Topic": "/session/status_text",
        })

    return {
        "Panels": panels,
        "Visualization Manager": {
            "Class": "",
            "Displays": displays,
            "Enabled": True,
            "Global Options": {
                "Background Color": "48; 48; 48",
                "Fixed Frame": "map",
                "Frame Rate": 30,
            },
            "Name": "root",
            "Tools": [
                {"Class": "rviz_default_plugins/Interact"},
                {"Class": "rviz_default_plugins/MoveCamera"},
                {"Class": "rviz_default_plugins/Select"},
                {"Class": "rviz_default_plugins/FocusCamera"},
                {"Class": "rviz_default_plugins/Measure", "Line color": "128; 128; 0"},
            ],
            "Views": {
                "Current": {
                    "Class": "rviz_default_plugins/Orbit",
                    "Distance": 8.0,
                    "Focal Point": {"X": 0.0, "Y": 0.0, "Z": 1.0},
                    "Name": "Current View",
                    "Near Clip Distance": 0.01,
                    "Pitch": 0.4,
                    "Target Frame": "map",
                    "Value": "Orbit (rviz_default_plugins)",
                    "Yaw": 3.14,
                },
                "Saved": None,
            },
        },
        "Window Geometry": {
            "Displays": {"collapsed": False},
            "Height": 1000,
            "Hide Left Dock": False,
            "Hide Right Dock": False,
            "Views": {"collapsed": False},
            "Width": 1600,
        },
    }


def write_rviz_config(
    path: str,
    videos: list[dict],
    depths: list[dict],
    markers: list[str] | None = None,
    status_panel: bool = False,
) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(
            build_rviz_config(videos, depths, markers, status_panel), handle,
            sort_keys=False, default_flow_style=False,
        )


# ── process management ──────────────────────────────────────────────────────


def _spawn(cmd: list[str]) -> subprocess.Popen:
    print("+ " + " ".join(cmd), flush=True)
    # Own session/process group so we can clean up the whole tree on exit.
    return subprocess.Popen(cmd, start_new_session=True)


def _terminate(procs: list[subprocess.Popen]) -> None:
    for proc in procs:
        if proc.poll() is not None:
            continue
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass
    deadline = time.time() + 5.0
    for proc in procs:
        try:
            proc.wait(timeout=max(0.0, deadline - time.time()))
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass


# ── main ────────────────────────────────────────────────────────────────────


def _describe(videos: list[dict], depths: list[dict]) -> None:
    if videos:
        print("Colour streams:")
        for spec in videos:
            print(f"  {os.path.basename(spec['video'])} -> {spec['publish_topic']}")
    else:
        print("Colour streams: (none)")
    if depths:
        print("Depth -> point clouds:")
        for spec in depths:
            print(f"  {spec['image']} + {spec['camera_info']} -> {spec['points']}")
    else:
        print("Depth streams: (none)")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="replay_session",
        description="Replay a recorded session (bag + videos + point clouds + RViz).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("session_dir", help="Session directory to replay")
    parser.add_argument("--rate", type=float, default=1.0, help="Bag playback speed")
    parser.add_argument("--loop", action="store_true", help="Loop bag playback")
    parser.add_argument("--no-video", action="store_true", help="Skip colour videos")
    parser.add_argument("--no-pointclouds", action="store_true", help="Skip point clouds")
    parser.add_argument("--no-rviz", action="store_true", help="Don't launch RViz")
    parser.add_argument("--rviz-config", default=None, help="RViz config path")
    parser.add_argument(
        "--list", action="store_true",
        help="Print discovered streams and exit (starts nothing)",
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

    bag_topics = load_bag_topics(bag_dir)
    videos = [] if args.no_video else discover_videos(session, load_expected(session))
    depths = [] if args.no_pointclouds else discover_depths(bag_topics)

    print(f"Session : {session}")
    print(f"Bag     : {bag_dir} ({len(bag_topics)} topics)")
    _describe(videos, depths)
    print()

    if args.list:
        rviz_config = args.rviz_config or os.path.join(session, "replay.rviz")
        print(f"Would start: bag play, {len(videos)} video stream(s), "
              f"{len(depths)} point cloud node(s)"
              + ("" if args.no_rviz else f", rviz ({rviz_config})"))
        return 0

    procs: list[subprocess.Popen] = []
    bag_proc: subprocess.Popen | None = None
    try:
        bag_proc = _spawn(bag_play_cmd(bag_dir, args.rate, args.loop))
        procs.append(bag_proc)

        if videos:
            procs.append(_spawn(video_publisher_cmd(videos)))
        for spec in depths:
            procs.append(_spawn(pointcloud_cmd(spec)))

        if not args.no_rviz:
            rviz_config = args.rviz_config or os.path.join(session, "replay.rviz")
            write_rviz_config(rviz_config, videos, depths)
            print(f"RViz config: {rviz_config}", flush=True)
            procs.append(_spawn(rviz_cmd(rviz_config)))

        # Bag playback paces everything; when it ends, the replay is over.
        while bag_proc.poll() is None:
            time.sleep(0.5)
        print("Bag playback finished.", flush=True)
    except KeyboardInterrupt:
        print("\nInterrupted.", flush=True)
    finally:
        _terminate(procs)

    return bag_proc.returncode if bag_proc is not None else 0


if __name__ == "__main__":
    sys.exit(main())
