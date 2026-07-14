#!/usr/bin/env python3
"""Launch the topic health monitor web dashboard.

Standalone from recording: start it once and leave it running. Reads the
`monitor:` block of recording.yaml (host/port/window/min_rate_factor) and
the sensor YAMLs in $SENSOR_CONFIG_DIR to know which topics to watch.

  ros2 launch session_recorder health_monitor.launch.py
Then open http://<rig-host>:<port>/ (default port 8765).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from session_recorder.topics import load_yaml


def _resolve_config(filename: str) -> str:
    config_dir = os.environ.get("SENSOR_CONFIG_DIR", "/home/ubuntu/config")
    return os.path.join(config_dir, filename)


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    recording_config = LaunchConfiguration("recording_config").perform(context)
    recording_raw = load_yaml(recording_config)
    recording_settings = (recording_raw or {}).get("recording_settings", {}) or {}
    monitor = recording_settings.get("monitor", {}) or {}

    if not monitor.get("enabled", True):
        return []

    return [
        Node(
            package="session_recorder",
            executable="topic_health_monitor",
            name="topic_health_monitor",
            output="screen",
            parameters=[{
                "host": str(monitor.get("host", "0.0.0.0")),
                "port": int(monitor.get("port", 8765)),
                "window_seconds": float(monitor.get("window_seconds", 5.0)),
                "min_rate_factor": float(monitor.get("min_rate_factor", 0.5)),
                "output_root": recording_settings.get(
                    "output_root", os.path.expanduser("~/data")
                ),
            }],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "recording_config",
            default_value=_resolve_config("recording.yaml"),
            description="Path to recording.yaml",
        ),
        OpaqueFunction(function=launch_setup),
    ])
