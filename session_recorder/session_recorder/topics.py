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
  bag_exclude_regex       - optional regex of topics to keep out of the bag
  bag_throttles           - "<in>:<out>:<hz>" specs for topic_throttle relays
  expected_topics         - "<topic>|<hz>|<sink>" manifest entries used by the
                            session sanity check and the health monitor
  expected_regex          - "<regex>|<hz>" manifest entries for regex-matched
                            topics (vicon)
  calib_source_dirs       - per-serial Kinect calibration dirs snapshot into
                            each session

A sensor participates when its camera entry has record: true (vicon, which
has no sensor YAML, is toggled in recording.yaml under vicon.enabled).

Raw vs rectified (why the defaults are what they are)
-----------------------------------------------------
Rectification/registration are derived, lossy transforms: they resample the
image and bake the current calibration into the pixels forever. The Kinect
only ever *measures* depth/IR at sd (512x424); the qhd/hd depth topics are
that same data registered into the colour frame and upsampled. We therefore
record the raw sd streams plus every relevant camera_info (which carries the
distortion model) and a snapshot of the bridge's calibration files (which
carry the depth<->colour extrinsic), so registered/rectified products can be
regenerated offline - identically today, or better after a future
calibration. Colour is the exception: it goes into H.265 video, where the
transform is baked in regardless, so it stays rectified for direct use.
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
    "depth": {"enabled": True, "mode": "raw", "fps": None},
}

REALSENSE_NATIVE_FPS = 30

# Topic naming differs by transport (`driver:` field per camera entry):
#
# driver: usb — standard realsense2_camera convention
#   (/<namespace>/<camera_name>/<stream>/<topic>, with
#   camera_namespace:=realsense, camera_name:=D555_<serial>):
#     /realsense/D555_<serial>/color/image_raw
#     /realsense/D555_<serial>/color/image_raw/compressed  (image_transport)
#     /realsense/D555_<serial>/infra1/image_rect_raw
#     /realsense/D555_<serial>/depth/image_rect_raw
#     /realsense/D555_<serial>/depth/image_rect_raw/compressedDepth (PNG)
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
        "depth": {
            "raw": "/depth/image_rect_raw",
            "compressed": "/depth/image_rect_raw/compressedDepth",
            "info_root": "/depth",
        },
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
        # The PoE firmware publishes no compressed depth; mode "compressed"
        # is rejected for dds cameras in realsense_setup.
        "depth": {"raw": "_Depth", "info_root": "_Depth"},
    },
}

DEFAULT_VELODYNE_TOPICS = ["velodyne_points"]
DEFAULT_VELODYNE_HZ = 10.0
DEFAULT_VICON_REGEX = "/vicon/.*"
# MarkerArray visualization topics are derived from the PointStamped topics
# and can be regenerated offline; keep them out of the bag by default.
DEFAULT_VICON_EXCLUDE_REGEX = "/vicon/.*visualization.*"
DEFAULT_VICON_HZ = 120.0


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


def expected_entry(topic: str, hz: float | None, sink: str) -> str:
    """Encode one manifest entry as "<topic>|<hz>|<sink>".

    hz None means presence-only: the sanity check requires messages but
    applies no rate threshold. sink is "bag" or "video".
    """
    hz_str = "" if hz is None else f"{float(hz):g}"
    return f"{topic}|{hz_str}|{sink}"


def parse_expected_entry(entry: str) -> tuple[str, float | None, str]:
    topic, hz_str, sink = entry.split("|")
    return topic, (float(hz_str) if hz_str else None), sink


def throttle_entry(source: str, hz: float) -> tuple[str, str]:
    """Return (out_topic, "<in>:<out>:<hz>" spec) for a throttled bag topic."""
    out = f"{source}/throttled"
    return out, f"{source}:{out}:{float(hz):g}"


DEFAULT_KINECT_STREAMS = {
    # Colour goes into H.265 video where the transform is baked in anyway;
    # keep it rectified so the videos are directly usable.
    "color": {
        "enabled": True,
        "resolution": "qhd",
        "rectified": True,
        "mode": "raw",
        "fps": 30,
    },
    # Depth/IR are stored as individual frames in the bag: record the raw
    # native-resolution sensor data so rectification/registration stay
    # reproducible offline (see module docstring).
    "depth": {
        "enabled": True,
        "resolution": "sd",
        "rectified": False,
        "mode": "compressed",
        "fps": 30,
    },
    "ir": {
        "enabled": True,
        "rectified": False,
        "mode": "compressed",
        "fps": 5,
    },
}

KINECT_NATIVE_FPS = 30
KINECT_COLOR_RESOLUTIONS = {"qhd", "hd", "sd"}
KINECT_DEPTH_RESOLUTIONS = {"qhd", "hd", "sd"}
DEFAULT_KINECT_CALIB_DIR = os.path.join(
    os.environ.get("BASE_WS", "/home/ubuntu/base_ws"),
    "src",
    "kinect2_ros2",
    "kinect2_bridge",
    "data",
)


def _kinect_image_topic(ns: str, res: str, stream: str, rectified: bool) -> str:
    """Build a kinect2_bridge image topic, validating the combination exists.

    The bridge (kinect2_bridge.cpp) publishes:
      sd : image_color_rect, image_ir[_rect], image_depth[_rect]
      qhd/hd : image_color[_rect], image_mono[_rect], image_depth_rect only
    i.e. unrectified depth/IR exist only at sd (that is the sensor's native
    data); qhd/hd depth is always the colour-registered product.
    """
    suffix = "_rect" if rectified else ""
    if stream == "color" and res == "sd" and not rectified:
        raise ValueError(
            "Kinect: sd colour only exists rectified (sd/image_color_rect)."
        )
    if stream == "depth" and res != "sd" and not rectified:
        raise ValueError(
            f"Kinect: unrectified depth only exists at sd; '{res}' depth is "
            "always the colour-registered image_depth_rect. Set resolution: sd "
            "or rectified: true."
        )
    return f"/{ns}/{res}/image_{stream}{suffix}"


def kinect_setup(cameras: list[dict], kinect_cfg: dict | None) -> dict:
    kinect_cfg = kinect_cfg or {}
    user_streams = kinect_cfg.get("streams", {}) or {}
    streams = {}
    for name, defaults in DEFAULT_KINECT_STREAMS.items():
        merged = dict(defaults)
        merged.update(user_streams.get(name, {}) or {})
        streams[name] = merged

    raw_topics: list[str] = []
    compressed_topics: list[str] = []
    fps_overrides: list[str] = []
    bag_topics: list[str] = []
    bag_throttles: list[str] = []
    expected: list[str] = []
    calib_dirs: list[str] = []

    calib_root = str(kinect_cfg.get("calibration_dir") or DEFAULT_KINECT_CALIB_DIR)
    snapshot_calib = bool(kinect_cfg.get("snapshot_calibration", True))

    for cam in cameras:
        ns = cam["name"]
        info_resolutions: set[str] = set()

        sc = streams["color"]
        if sc.get("enabled", True):
            res = str(sc.get("resolution", "qhd"))
            if res not in KINECT_COLOR_RESOLUTIONS:
                raise ValueError(
                    f"Kinect {ns}: color resolution '{res}' not valid. "
                    f"Choose from: {sorted(KINECT_COLOR_RESOLUTIONS)}"
                )
            base = _kinect_image_topic(ns, res, "color", sc.get("rectified", True))
            topic = f"{base}/compressed" if sc.get("mode") == "compressed" else base
            if sc.get("mode") == "compressed":
                compressed_topics.append(topic)
            else:
                raw_topics.append(topic)
            fps = sc.get("fps")
            if fps:
                fps_overrides.append(f"{topic}:{fps}")
            expected.append(expected_entry(topic, fps or KINECT_NATIVE_FPS, "video"))
            info_resolutions.add(res)

        sd_ = streams["depth"]
        if sd_.get("enabled", True):
            res = str(sd_.get("resolution", "sd"))
            if res not in KINECT_DEPTH_RESOLUTIONS:
                raise ValueError(
                    f"Kinect {ns}: depth resolution '{res}' not valid. "
                    f"Choose from: {sorted(KINECT_DEPTH_RESOLUTIONS)}"
                )
            base = _kinect_image_topic(ns, res, "depth", sd_.get("rectified", False))
            topic = f"{base}/compressed" if sd_.get("mode") == "compressed" else base
            fps = sd_.get("fps")
            if fps and fps < KINECT_NATIVE_FPS:
                topic, spec = throttle_entry(topic, fps)
                bag_throttles.append(spec)
            bag_topics.append(topic)
            expected.append(expected_entry(topic, fps or KINECT_NATIVE_FPS, "bag"))
            info_resolutions.add(res)

        si = streams["ir"]
        if si.get("enabled", True):
            base = _kinect_image_topic(ns, "sd", "ir", si.get("rectified", False))
            topic = f"{base}/compressed" if si.get("mode") == "compressed" else base
            fps = si.get("fps")
            if fps and fps < KINECT_NATIVE_FPS:
                topic, spec = throttle_entry(topic, fps)
                bag_throttles.append(spec)
            bag_topics.append(topic)
            expected.append(expected_entry(topic, fps or KINECT_NATIVE_FPS, "bag"))
            info_resolutions.add("sd")

        # camera_info for every resolution in use: carries K and the
        # distortion model D, without which raw streams cannot be
        # rectified/registered later.
        for res in sorted(info_resolutions):
            info = f"/{ns}/{res}/camera_info"
            bag_topics.append(info)
            expected.append(expected_entry(info, KINECT_NATIVE_FPS, "bag"))

        # The bridge's calibration files hold the depth<->colour extrinsic
        # (calib_pose.yaml), needed to reproduce registration offline.
        serial = str(cam.get("serial", "")).strip()
        if snapshot_calib and serial:
            calib_dirs.append(os.path.join(calib_root, serial))

    return {
        "color_topics": raw_topics,
        "color_compressed_topics": compressed_topics,
        "topic_fps_overrides": fps_overrides,
        "bag_topics": bag_topics,
        "bag_throttles": bag_throttles,
        "expected_topics": expected,
        "calib_source_dirs": calib_dirs,
    }


def realsense_setup(cameras: list[dict], user_streams: dict | None) -> dict:
    streams = _merge_streams(user_streams)
    raw_topics: list[str] = []
    raw_remaps: list[str] = []
    compressed_topics: list[str] = []
    fps_overrides: list[str] = []
    bag_topics: list[str] = []
    bag_throttles: list[str] = []
    expected: list[str] = []

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
                expected.append(
                    expected_entry(topic, fps or REALSENSE_NATIVE_FPS, "video")
                )
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
                    expected.append(
                        expected_entry(sub, fps or REALSENSE_NATIVE_FPS, "video")
                    )
                else:
                    raw_topics.append(source)
                    if fps:
                        fps_overrides.append(f"{source}:{fps}")
                    expected.append(
                        expected_entry(source, fps or REALSENSE_NATIVE_FPS, "video")
                    )

            # camera_info/metadata rates vary by firmware; presence-only.
            for aux in (f"{info_root}/camera_info", f"{info_root}/metadata"):
                bag_topics.append(aux)
                expected.append(expected_entry(aux, None, "bag"))

        depth_cfg = streams["depth"]
        if depth_cfg.get("enabled", True):
            depth_map = suffix_by_stream["depth"]
            mode = depth_cfg.get("mode", "raw")
            if mode not in depth_map:
                raise ValueError(
                    f"Stream 'depth' does not support mode '{mode}' with "
                    f"driver '{driver}' (the PoE firmware publishes no "
                    "compressed depth)."
                )
            depth_topic = f"{prefix}{depth_map[mode]}"
            fps = depth_cfg.get("fps")
            if fps and fps < REALSENSE_NATIVE_FPS:
                depth_topic, spec = throttle_entry(depth_topic, fps)
                bag_throttles.append(spec)
            depth_info_root = f"{prefix}{depth_map['info_root']}"
            bag_topics.append(depth_topic)
            expected.append(
                expected_entry(depth_topic, fps or REALSENSE_NATIVE_FPS, "bag")
            )
            for aux in (
                f"{depth_info_root}/camera_info",
                f"{depth_info_root}/metadata",
            ):
                bag_topics.append(aux)
                expected.append(expected_entry(aux, None, "bag"))

    return {
        "color_topics": raw_topics,
        "color_remaps": raw_remaps,
        "color_compressed_topics": compressed_topics,
        "topic_fps_overrides": fps_overrides,
        "bag_topics": bag_topics,
        "bag_throttles": bag_throttles,
        "expected_topics": expected,
    }


def velodyne_setup(velodyne_cfg: dict, recording_cfg: dict) -> dict:
    """Per-unit bag topics. Configured names are relative to each unit's
    namespace (/<name>/<topic>); absolute names (leading /) pass through
    unchanged for backward compatibility."""
    units = recorded_cameras(velodyne_cfg)
    if not units:
        return {"bag_topics": [], "expected_topics": []}
    vel_cfg = recording_cfg.get("velodyne", {}) or {}
    names = vel_cfg.get("topics", DEFAULT_VELODYNE_TOPICS)
    hz = vel_cfg.get("expected_hz", DEFAULT_VELODYNE_HZ)
    topics: list[str] = []
    expected: list[str] = []
    for unit in units:
        for name in names:
            name = str(name)
            if not name:
                continue
            topic = name if name.startswith("/") else f"/{unit['name']}/{name}"
            topics.append(topic)
            expected.append(expected_entry(topic, hz, "bag"))
    return {"bag_topics": topics, "expected_topics": expected}


def vicon_regex(recording_cfg: dict) -> str:
    vicon = recording_cfg.get("vicon", {}) or {}
    if not vicon.get("enabled", False):
        return ""
    return str(vicon.get("topic_regex", DEFAULT_VICON_REGEX))


def vicon_exclude_regex(recording_cfg: dict) -> str:
    vicon = recording_cfg.get("vicon", {}) or {}
    if not vicon.get("enabled", False):
        return ""
    return str(vicon.get("exclude_regex", DEFAULT_VICON_EXCLUDE_REGEX))


def vicon_expected(recording_cfg: dict) -> list[str]:
    vicon = recording_cfg.get("vicon", {}) or {}
    if not vicon.get("enabled", False):
        return []
    regex = str(vicon.get("topic_regex", DEFAULT_VICON_REGEX))
    hz = vicon.get("expected_hz", DEFAULT_VICON_HZ)
    hz_str = "" if hz is None else f"{float(hz):g}"
    return [f"{regex}|{hz_str}"]


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
        "bag_exclude_regex": "",
        "bag_throttles": [],
        "expected_topics": [
            expected_entry("/tf", None, "bag"),
            expected_entry("/tf_static", None, "bag"),
        ],
        "expected_regex": [],
        "calib_source_dirs": [],
    }

    kinect = kinect_setup(
        recorded_cameras(kinect_cfg), recording_settings.get("kinect")
    )
    for key in (
        "color_topics",
        "color_compressed_topics",
        "topic_fps_overrides",
        "bag_topics",
        "bag_throttles",
        "expected_topics",
        "calib_source_dirs",
    ):
        setup[key] += kinect.get(key, [])

    realsense = realsense_setup(
        recorded_cameras(realsense_cfg), recording_settings.get("realsense")
    )
    for key in (
        "color_topics",
        "color_remaps",
        "color_compressed_topics",
        "topic_fps_overrides",
        "bag_topics",
        "bag_throttles",
        "expected_topics",
    ):
        setup[key] += realsense[key]

    velodyne = velodyne_setup(velodyne_cfg, recording_settings)
    setup["bag_topics"] += velodyne["bag_topics"]
    setup["expected_topics"] += velodyne["expected_topics"]

    for extra in recording_settings.get("bag_extra_topics", []) or []:
        setup["bag_topics"].append(str(extra))
        setup["expected_topics"].append(expected_entry(str(extra), None, "bag"))

    setup["bag_regex"] = vicon_regex(recording_settings)
    setup["bag_exclude_regex"] = vicon_exclude_regex(recording_settings)
    setup["expected_regex"] = vicon_expected(recording_settings)

    # De-duplicate while preserving order.
    for key in (
        "color_topics",
        "color_compressed_topics",
        "bag_topics",
        "bag_throttles",
        "expected_topics",
        "calib_source_dirs",
    ):
        setup[key] = list(dict.fromkeys(setup[key]))

    return setup
