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

    watchdog_cfg = recording_settings.get("watchdog", {}) or {}

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
                    # Param types here must match the node's declarations, which
                    # rclpy infers from the defaults in hardening.py: video_crf
                    # is INTEGER (18), video_fps is DOUBLE (30.0). Casting
                    # explicitly means a YAML edit such as `video_crf: 18.0`
                    # cannot kill the node the way bag_max_size_mb did.
                    "video_crf": int(recording_settings.get("video_crf", 18)),
                    "video_fps": float(recording_settings.get("video_fps", 30.0)),
                    # Software encoder fallback: without it an unusable NVENC
                    # silently produces 0-byte videos (2026-10-01).
                    "video_software_fallback": bool(
                        recording_settings.get("video_software_fallback", True)
                    ),
                    "video_encoder_probe": bool(
                        recording_settings.get("video_encoder_probe", True)
                    ),
                    "video_startup_grace_s": float(
                        recording_settings.get("video_startup_grace_s", 20.0)
                    ),
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
                    # Split the bag so one crash costs at most one split.
                    # INTEGER, not float: the node declares this from an int
                    # default (hardening.DEFAULT_BAG_MAX_SIZE_MB = 1024), so an
                    # override of 1024.0 was rejected at construction with
                    # InvalidParameterTypeException and the manager died before
                    # it could start. Every other split/grace value is declared
                    # from a float default, which is why only this one broke.
                    "bag_max_size_mb": int(
                        recording_settings.get("bag_max_size_mb", 1024)
                    ),
                    "bag_max_duration_s": float(
                        recording_settings.get("bag_max_duration_s", 300)
                    ),
                    "stop_grace_s": float(
                        recording_settings.get("stop_grace_s", 30)
                    ),
                    "watchdog_enabled": bool(watchdog_cfg.get("enabled", True)),
                    "watchdog_interval_s": float(
                        watchdog_cfg.get("interval_s", 5.0)
                    ),
                    "watchdog_restart_bag": bool(
                        watchdog_cfg.get("restart_bag", True)
                    ),
                    "watchdog_restart_throttle": bool(
                        watchdog_cfg.get("restart_throttle", True)
                    ),
                    "nexus_capture": nexus_capture,
                    "nexus_host": nexus_host,
                    "nexus_port": nexus_port,
                    "nexus_path": nexus_path,
                }
            ],
        )
    ]


def _align_nvidia_driver_libs() -> None:
    """Point the container's driver userspace at the host kernel module version.

    The image installs Ubuntu's libnvidia-compute (580.178.04 here) while this
    host runs the Pop!_OS driver (580.173.02). The container runtime injects the
    host's libcuda/libnvidia-ml as well, but does not overwrite the .so.1
    symlinks the image created, so the loader picks the mismatched userspace:
    NVML reports a driver/library mismatch and libcuda's forward-compat path
    fails on consumer GeForce hardware, which made every colour stream record 0
    bytes on 2026-10-01.

    Doing this here -- synchronously, while the launch description is built --
    guarantees it happens before the recording manager node constructs and runs
    its encoder probe, whatever shell or entry point invoked the launch. The
    script is idempotent and non-fatal: any failure is reported and ignored so a
    broken workaround can never stop the rig from coming up (the encoder probe
    and CPU fallback still decide correctly on their own).
    """
    script = os.path.join(
        os.environ.get("BASH_SCRIPTS_DIR", "/home/ubuntu/bash_scripts"),
        "align_nvidia_driver_libs.sh",
    )
    if not os.path.isfile(script):
        return
    try:
        import subprocess

        proc = subprocess.run(
            ["bash", script, "--quiet"],
            capture_output=True,
            text=True,
            timeout=20,
        )
        if proc.stdout.strip():
            print(f"[unified_recording] {proc.stdout.strip()}")
    except Exception as exc:  # never let a housekeeping step break the launch
        print(f"[unified_recording] driver-library alignment skipped: {exc}")


def generate_launch_description():
    base_ws = os.environ.get("BASE_WS", "/home/ubuntu/base_ws")

    # Must run before the node is constructed: the manager probes the encoder as
    # part of its own startup and would otherwise fall back to the CPU encoder.
    _align_nvidia_driver_libs()

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
