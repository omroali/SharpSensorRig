"""Derive recording topics for every sensor from the unified config directory.

Reads the per-sensor YAMLs in $SENSOR_CONFIG_DIR (kinect_cameras.yaml,
realsense_cameras.yaml, velodyne.yaml) plus recording.yaml, and produces the
parameter set the unified recording_manager expects:

  color_topics            - raw Image topics for colour_video_recorder
  color_remaps            - ros2-style remaps applied to that recorder
  color_compressed_topics - CompressedImage topics for the same recorder
  topic_fps_overrides     - "<topic>:<fps>" subsampling entries
  bag_topics              - everything ros2 bag record subscribes to by name
  bag_regex               - optional regex of additional bag topics (vicon)

A sensor participates when its camera entry has record: true (vicon, which
has no sensor YAML, is toggled in recording.yaml under vicon.enabled).
"""

from __future__ import annotations

import os
from typing import Any

import yaml

# RealSense per-stream defaults; recording.yaml `streams:` overrides these.
DEFAULT_REALSENSE_STREAMS = {
    "color": {"enabled": True, "mode": "compressed", "fps": 30},
    "infrared_1": {"enabled": True, "mode": "raw", "fps": 30},
    "infrared_2": {"enabled": True, "mode": "raw", "fps": 30},
    "depth": {"enabled": True},
}

# Topic naming differs by transport (`driver:` field per camera entry):
#
# driver: usb — standard realsense2_camera convention
#   (/<namespace>/<camera_name>/<stream>/<topic>, with
#   camera_namespace:=realsense, camera_name:=D555_<serial>):
#     /realsense/D555_<serial>/color/image_raw
#     /realsense/D555_<serial>/color/image_raw/compressed  (image_transport)
#     /realsense/D555_<serial>/infra1/image_rect_raw
#     /realsense/D555_<serial>/depth/image_rect_raw
#
# driver: dds — flat convention published by the PoE D555 firmware itself;
#   we subscribe to the camera's topics directly (no relay in the data path):
#     /realsense/D555_<serial>_Color            (+ /camera_info, /metadata)
#     /realsense/D555_<serial>_CompressedColor  (pre-encoded JPEG)
#     /realsense/D555_<serial>_Infrared_1
#     /realsense/D555_<serial>_Depth
#
# Each stream maps to:
#   raw/compressed — the image topic to subscribe to
#   info_root      — prefix under which camera_info and metadata live
#   sub_suffix     — (dds raw only) subscribe via <topic><sub_suffix> with a
#                    remap back to the source; kept from the proven PoE setup
#                    so recorded filenames stay consistent across sessions.
_STREAM_SUFFIX_BY_DRIVER = {
    "usb": {
        "color": {
            "raw": "/color/image_raw",
            "compressed": "/color/image_raw/compressed",
            "info_root": "/color",
        },
        "infrared_1": {"raw": "/infra1/image_rect_raw", "info_root": "/infra1"},
        "infrared_2": {"raw": "/infra2/image_rect_raw", "info_root": "/infra2"},
        "depth": {"raw": "/depth/image_rect_raw", "info_root": "/depth"},
    },
    "dds": {
        "color": {
            "raw": "_Color",
            "compressed": "_CompressedColor",
            "info_root": "_Color",
            "sub_suffix": "/image",
        },
        "infrared_1": {
            "raw": "_Infrared_1",
            "info_root": "_Infrared_1",
            "sub_suffix": "/image",
        },
        "infrared_2": {
            "raw": "_Infrared_2",
            "info_root": "_Infrared_2",
            "sub_suffix": "/image",
        },
        "depth": {"raw": "_Depth", "info_root": "_Depth"},
    },
}

DEFAULT_VELODYNE_TOPICS = ["velodyne_points"]
DEFAULT_VICON_REGEX = "/vicon/.*"


def config_dir() -> str:
    return os.environ.get("SENSOR_CONFIG_DIR", "/home/ubuntu/config")


def load_yaml(path: str) -> dict:
    if not path or not os.path.isfile(path):
        return {}
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def recorded_cameras(cfg: dict) -> list[dict]:
    """Camera entries with record: true, each given a `name` key."""
    cameras = cfg.get("cameras", {}) or {}
    selected = []
    for name, camera in cameras.items():
        if not isinstance(camera, dict) or not camera.get("record", False):
            continue
        entry = dict(camera)
        entry["name"] = str(name)
        selected.append(entry)
    return selected


def _merge_streams(user_streams: dict | None) -> dict:
    resolved = {}
    user_streams = user_streams or {}
    for name, defaults in DEFAULT_REALSENSE_STREAMS.items():
        merged = dict(defaults)
        merged.update(user_streams.get(name, {}) or {})
        resolved[name] = merged
    return resolved


DEFAULT_KINECT_STREAMS = {
    "color": {"enabled": True, "resolution": "qhd", "mode": "raw", "fps": 30},
    "depth": {"enabled": True, "resolution": "qhd", "mode": "compressed"},
    "ir": {"enabled": True, "mode": "compressed"},
}

KINECT_COLOR_RESOLUTIONS = {"qhd", "hd", "sd"}
KINECT_DEPTH_RESOLUTIONS = {"qhd", "hd"}


def kinect_setup(cameras: list[dict], kinect_cfg: dict | None) -> dict:
    user_streams = (kinect_cfg or {}).get("streams", {}) or {}
    streams = {}
    for name, defaults in DEFAULT_KINECT_STREAMS.items():
        merged = dict(defaults)
        merged.update(user_streams.get(name, {}) or {})
        streams[name] = merged

    raw_topics: list[str] = []
    compressed_topics: list[str] = []
    fps_overrides: list[str] = []
    bag_topics: list[str] = []

    for cam in cameras:
        ns = cam["name"]

        sc = streams["color"]
        if sc.get("enabled", True):
            res = str(sc.get("resolution", "qhd"))
            if res not in KINECT_COLOR_RESOLUTIONS:
                raise ValueError(
                    f"Kinect {ns}: color resolution '{res}' not valid. "
                    f"Choose from: {sorted(KINECT_COLOR_RESOLUTIONS)}"
                )
            base = f"/{ns}/{res}/image_color_rect"
            topic = f"{base}/compressed" if sc.get("mode") == "compressed" else base
            if sc.get("mode") == "compressed":
                compressed_topics.append(topic)
            else:
                raw_topics.append(topic)
            fps = sc.get("fps")
            if fps:
                fps_overrides.append(f"{topic}:{fps}")

        sd = streams["depth"]
        if sd.get("enabled", True):
            res = str(sd.get("resolution", "qhd"))
            if res not in KINECT_DEPTH_RESOLUTIONS:
                raise ValueError(
                    f"Kinect {ns}: depth resolution '{res}' not valid. "
                    f"Choose from: {sorted(KINECT_DEPTH_RESOLUTIONS)}"
                )
            depth_base = f"/{ns}/{res}/image_depth_rect"
            if sd.get("mode") == "compressed":
                bag_topics.append(f"{depth_base}/compressed")
            else:
                bag_topics.append(depth_base)

        si = streams["ir"]
        if si.get("enabled", True):
            ir_base = f"/{ns}/sd/image_ir_rect"
            if si.get("mode") == "compressed":
                bag_topics.append(f"{ir_base}/compressed")
            else:
                bag_topics.append(ir_base)

        res = streams["color"].get("resolution", "qhd")
        bag_topics.append(f"/{ns}/{res}/camera_info")

    return {
        "color_topics": raw_topics,
        "color_compressed_topics": compressed_topics,
        "topic_fps_overrides": fps_overrides,
        "bag_topics": bag_topics,
    }


def realsense_setup(cameras: list[dict], user_streams: dict | None) -> dict:
    streams = _merge_streams(user_streams)
    raw_topics: list[str] = []
    raw_remaps: list[str] = []
    compressed_topics: list[str] = []
    fps_overrides: list[str] = []
    bag_topics: list[str] = []

    for camera in cameras:
        serial = camera.get("serial")
        if not serial:
            raise ValueError(f"Camera {camera.get('name', '<unknown>')} missing serial")
        driver = str(camera.get("driver", "dds")).lower()
        if driver not in _STREAM_SUFFIX_BY_DRIVER:
            raise ValueError(
                f"Camera {camera['name']}: unknown driver '{driver}'. "
                f"Valid drivers: {sorted(_STREAM_SUFFIX_BY_DRIVER)}"
            )
        suffix_by_stream = _STREAM_SUFFIX_BY_DRIVER[driver]
        model = camera.get("model", "D555")
        prefix = f"/realsense/{model}_{serial}"

        for stream_name in ("color", "infrared_1", "infrared_2"):
            cfg = streams[stream_name]
            if not cfg.get("enabled", True):
                continue
            mode = cfg.get("mode", "raw")
            suffix_map = suffix_by_stream[stream_name]
            if mode not in suffix_map:
                raise ValueError(
                    f"Stream '{stream_name}' does not support mode '{mode}' "
                    f"with driver '{driver}'."
                )

            fps = cfg.get("fps")
            info_root = f"{prefix}{suffix_map['info_root']}"

            if mode == "compressed":
                topic = f"{prefix}{suffix_map['compressed']}"
                compressed_topics.append(topic)
                if fps:
                    fps_overrides.append(f"{topic}:{fps}")
            else:
                source = f"{prefix}{suffix_map['raw']}"
                sub_suffix = suffix_map.get("sub_suffix")
                if sub_suffix:
                    # Subscribe via an alias remapped to the source topic so
                    # output filenames match the established PoE sessions.
                    sub = f"{source}{sub_suffix}"
                    raw_topics.append(sub)
                    raw_remaps.append(f"{sub}:={source}")
                    if fps:
                        fps_overrides.append(f"{sub}:{fps}")
                else:
                    raw_topics.append(source)
                    if fps:
                        fps_overrides.append(f"{source}:{fps}")

            bag_topics += [f"{info_root}/camera_info", f"{info_root}/metadata"]

        if streams["depth"].get("enabled", True):
            depth_map = suffix_by_stream["depth"]
            depth_topic = f"{prefix}{depth_map['raw']}"
            depth_info_root = f"{prefix}{depth_map['info_root']}"
            bag_topics += [
                depth_topic,
                f"{depth_info_root}/camera_info",
                f"{depth_info_root}/metadata",
            ]

    return {
        "color_topics": raw_topics,
        "color_remaps": raw_remaps,
        "color_compressed_topics": compressed_topics,
        "topic_fps_overrides": fps_overrides,
        "bag_topics": bag_topics,
    }


def velodyne_setup(velodyne_cfg: dict, recording_cfg: dict) -> list[str]:
    """Per-unit bag topics. Configured names are relative to each unit's
    namespace (/<name>/<topic>); absolute names (leading /) pass through
    unchanged for backward compatibility."""
    units = recorded_cameras(velodyne_cfg)
    if not units:
        return []
    names = (recording_cfg.get("velodyne", {}) or {}).get(
        "topics", DEFAULT_VELODYNE_TOPICS
    )
    topics: list[str] = []
    for unit in units:
        for name in names:
            name = str(name)
            if not name:
                continue
            topics.append(name if name.startswith("/") else f"/{unit['name']}/{name}")
    return topics


def vicon_regex(recording_cfg: dict) -> str:
    vicon = recording_cfg.get("vicon", {}) or {}
    if not vicon.get("enabled", False):
        return ""
    return str(vicon.get("topic_regex", DEFAULT_VICON_REGEX))


def build_recording_setup(
    kinect_cfg: dict,
    realsense_cfg: dict,
    velodyne_cfg: dict,
    recording_settings: dict,
) -> dict[str, Any]:
    """Merge every sensor's topic set into one recording_manager parameter dict."""
    setup = {
        "color_topics": [],
        "color_remaps": [],
        "color_compressed_topics": [],
        "topic_fps_overrides": [],
        "bag_topics": ["/tf", "/tf_static"],
        "bag_regex": "",
    }

    kinect = kinect_setup(
        recorded_cameras(kinect_cfg), recording_settings.get("kinect")
    )
    for key in (
        "color_topics",
        "color_compressed_topics",
        "topic_fps_overrides",
        "bag_topics",
    ):
        setup[key] += kinect.get(key, [])

    realsense = realsense_setup(
        recorded_cameras(realsense_cfg), recording_settings.get("streams")
    )
    for key in (
        "color_topics",
        "color_remaps",
        "color_compressed_topics",
        "topic_fps_overrides",
        "bag_topics",
    ):
        setup[key] += realsense[key]

    setup["bag_topics"] += velodyne_setup(velodyne_cfg, recording_settings)
    setup["bag_topics"] += [str(t) for t in recording_settings.get("bag_extra_topics", []) or []]
    setup["bag_regex"] = vicon_regex(recording_settings)

    # De-duplicate while preserving order.
    for key in ("color_topics", "color_compressed_topics", "bag_topics"):
        setup[key] = list(dict.fromkeys(setup[key]))

    return setup
