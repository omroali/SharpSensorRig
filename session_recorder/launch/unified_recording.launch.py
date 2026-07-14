#!/usr/bin/env python3
"""Launch the unified recording manager without starting capture.

One launch file for every sensor. Reads kinect_cameras.yaml,
realsense_cameras.yaml, velodyne.yaml, and recording.yaml from
$SENSOR_CONFIG_DIR (default /home/ubuntu/config), derives the full
topic set, and starts the recording_manager service node.

Use /start_recording and /stop_recording services to control capture.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from session_recorder.topics import build_recording_setup, load_yaml


def _resolve_config(filename: str, fallback: str) -> str:
    """Prefer $SENSOR_CONFIG_DIR/<filename>; fall back to the given path."""
    config_dir = os.environ.get("SENSOR_CONFIG_DIR", "/home/ubuntu/config")
    candidate = os.path.join(config_dir, filename)
    if os.path.isfile(candidate):
        return candidate
    return fallback


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    kinect_config = LaunchConfiguration("kinect_config").perform(context)
    realsense_config = LaunchConfiguration("realsense_config").perform(context)
    velodyne_config = LaunchConfiguration("velodyne_config").perform(context)
    recording_config = LaunchConfiguration("recording_config").perform(context)

    # Load all sensor configs (gracefully handles missing files via load_yaml).
    kinect_cfg = load_yaml(kinect_config)
    realsense_cfg = load_yaml(realsense_config)
    velodyne_cfg = load_yaml(velodyne_config)

    recording_raw = load_yaml(recording_config)
    recording_settings = (
        recording_raw.get("recording_settings", {}) if recording_raw else {}
    )

    setup = build_recording_setup(
        kinect_cfg, realsense_cfg, velodyne_cfg, recording_settings
    )

    vicon_cfg = recording_settings.get("vicon", {}) or {}
    nexus_capture = bool(vicon_cfg.get("nexus_capture", False))
    nexus_host = str(vicon_cfg.get("nexus_host", "192.168.10.1"))
    nexus_port = int(vicon_cfg.get("nexus_port", 3030))
    nexus_path = str(vicon_cfg.get("nexus_path", ""))

    # launch_ros can't infer the element type of an empty STRING_ARRAY
    # (it ends up as () and fails validation). Pad with a single empty
    # string; the manager filters falsy entries on read.
    def non_empty(values):
        return values if values else [""]

    return [
        Node(
            package="session_recorder",
            executable="recording_manager",
            name="recording_manager",
            output="screen",
            parameters=[
                {
                    "output_root": recording_settings.get(
                        "output_root", os.path.expanduser("~/data")
                    ),
                    "participant_id": recording_settings.get("uuid", "0000"),
                    "color_topics": non_empty(setup["color_topics"]),
                    "color_remaps": non_empty(setup["color_remaps"]),
                    "color_compressed_topics": non_empty(
                        setup["color_compressed_topics"]
                    ),
                    "topic_fps_overrides": non_empty(setup["topic_fps_overrides"]),
                    "bag_topics": non_empty(setup["bag_topics"]),
                    "bag_regex": setup.get("bag_regex", ""),
                    "bag_exclude_regex": setup.get("bag_exclude_regex", ""),
                    "bag_throttles": non_empty(setup.get("bag_throttles", [])),
                    "expected_topics": non_empty(setup.get("expected_topics", [])),
                    "expected_regex": non_empty(setup.get("expected_regex", [])),
                    "calib_source_dirs": non_empty(
                        setup.get("calib_source_dirs", [])
                    ),
                    "session_check_enabled": bool(
                        (recording_settings.get("session_check", {}) or {}).get(
                            "enabled", True
                        )
                    ),
                    "session_check_min_rate_factor": float(
                        (recording_settings.get("session_check", {}) or {}).get(
                            "min_rate_factor", 0.5
                        )
                    ),
                    "video_encoder": recording_settings.get(
                        "video_encoder", "hevc_nvenc"
                    ),
                    "video_crf": recording_settings.get("video_crf", 18),
                    "video_fps": recording_settings.get("video_fps", 30.0),
                    "bag_storage": recording_settings.get("bag_storage", "mcap"),
                    "bag_compression": recording_settings.get(
                        "bag_compression", "none"
                    ),
                    "mcap_preset": recording_settings.get(
                        "mcap_preset", "zstd_fast"
                    ),
                    "bag_cache_size_mb": int(
                        recording_settings.get("bag_cache_size_mb", 128)
                    ),
                    "nexus_capture": nexus_capture,
                    "nexus_host": nexus_host,
                    "nexus_port": nexus_port,
                    "nexus_path": nexus_path,
                }
            ],
        )
    ]


def generate_launch_description():
    base_ws = os.environ.get("BASE_WS", "/home/ubuntu/base_ws")

    # Fallback paths point at the old per-package configs so the launch
    # still works outside the container / without SENSOR_CONFIG_DIR.
    kinect_fallback = os.path.join(
        base_ws,
        "src",
        "kinect2_ros2",
        "kinect2_bridge",
        "config",
        "multi_camera_config.yaml",
    )
    realsense_fallback = os.path.join(
        base_ws,
        "src",
        "realsense_ros2",
        "realsense_config.yaml",
    )
    # velodyne.yaml and recording.yaml were created in Phase 2; they live
    # only in the unified config dir, so the fallback is that same dir.
    config_dir = os.environ.get("SENSOR_CONFIG_DIR", "/home/ubuntu/config")
    velodyne_fallback = os.path.join(config_dir, "velodyne.yaml")
    recording_fallback = os.path.join(config_dir, "recording.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "kinect_config",
                default_value=_resolve_config("kinect_cameras.yaml", kinect_fallback),
                description="Path to kinect_cameras.yaml",
            ),
            DeclareLaunchArgument(
                "realsense_config",
                default_value=_resolve_config(
                    "realsense_cameras.yaml", realsense_fallback
                ),
                description="Path to realsense_cameras.yaml",
            ),
            DeclareLaunchArgument(
                "velodyne_config",
                default_value=_resolve_config("velodyne.yaml", velodyne_fallback),
                description="Path to velodyne.yaml",
            ),
            DeclareLaunchArgument(
                "recording_config",
                default_value=_resolve_config("recording.yaml", recording_fallback),
                description="Path to recording.yaml",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
