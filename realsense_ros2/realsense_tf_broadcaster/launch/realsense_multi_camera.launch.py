#!/usr/bin/env python3
"""Multi-camera RealSense launch — reads realsense_cameras.yaml, spawns
camera nodes per enabled camera.

Supports two driver modes per camera entry:
  driver: usb  — launches realsense2_camera_node (standard ROS2 driver)
  driver: dds  — PoE / network-attached camera (uses topic_relay)

Topics follow the standard realsense2_camera convention:
  /realsense/D555_<serial>/color/image_raw
  /realsense/D555_<serial>/depth/image_rect_raw
  /realsense/D555_<serial>/infra1/image_rect_raw
  /realsense/D555_<serial>/infra2/image_rect_raw
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _config_path():
    config_dir = os.environ.get("SENSOR_CONFIG_DIR", "")
    if config_dir:
        candidate = os.path.join(config_dir, "realsense_cameras.yaml")
        if os.path.isfile(candidate):
            return candidate
    base = os.environ.get("BASE_WS", "/home/ubuntu/base_ws")
    return os.path.join(base, "src", "realsense_ros2", "realsense_config.yaml")


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    config_file = LaunchConfiguration("config_file").perform(context)

    if not os.path.isfile(config_file):
        return [LogInfo(msg=f"Config not found: {config_file}")]

    with open(config_file, "r") as f:
        cfg = yaml.safe_load(f) or {}

    cameras = cfg.get("cameras", {}) or {}
    world_frame = cfg.get("world_frame", "map")

    actions = [LogInfo(msg=f"Loading {len(cameras)} camera(s) from {config_file}")]

    delay = 0.0
    delay_step = 8.0

    for cam_name, cam_cfg in cameras.items():
        if not isinstance(cam_cfg, dict):
            continue
        if not cam_cfg.get("enabled", False):
            actions.append(LogInfo(msg=f"Skipping {cam_name}: disabled"))
            continue

        serial = str(cam_cfg.get("serial", ""))
        model = str(cam_cfg.get("model", "D555"))
        namespace = cam_cfg.get("namespace", cam_name)
        frame = cam_cfg.get("frame", f"{namespace}_link")

        if not serial:
            actions.append(LogInfo(msg=f"Skipping {cam_name}: no serial"))
            continue

        driver = cam_cfg.get("driver", "dds")
        if driver == "usb":
            # ── USB camera: launch realsense2_camera_node ──────────────
            # Standard topic convention — no remapping needed.
            # TF frames are published by the node itself (publish_tf:=true).
            node = Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                name=f"{model}_{serial}",
                namespace="realsense",
                output="screen",
                parameters=[
                    {
                        "serial_no": f"_{serial}",
                        "base_frame_id": frame,
                        "publish_tf": True,
                        "enable_color": True,
                        "enable_depth": True,
                        "enable_infra1": False,
                        "enable_infra2": False,
                        "enable_gyro": False,
                        "enable_accel": False,
                        "enable_sync": False,
                        "align_depth.enable": False,
                        "pointcloud.enable": False,
                        "initial_reset": False,
                        "rgb_camera.color_profile": "640x480x15",
                        "depth_module.depth_profile": "640x360x15",
                    }
                ],
            )
            if delay > 0:
                actions.append(TimerAction(period=delay, actions=[node]))
            else:
                actions.append(node)
            delay += delay_step

            # TF: map → camera_link (published here so it survives
            # even if realsense2_camera is restarted)
            pos = cam_cfg.get("position", {}) or {}
            ori = cam_cfg.get("orientation", {}) or {}
            actions.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"{cam_name}_map_tf",
                    output="screen",
                    arguments=[
                        "--frame-id",
                        world_frame,
                        "--child-frame-id",
                        frame,
                        "--x",
                        str(pos.get("x", 0)),
                        "--y",
                        str(pos.get("y", 0)),
                        "--z",
                        str(pos.get("z", 0)),
                        "--roll",
                        str(ori.get("roll", 0)),
                        "--pitch",
                        str(ori.get("pitch", 0)),
                        "--yaw",
                        str(ori.get("yaw", 0)),
                    ],
                )
            )
            continue

        # ── DDS / PoE camera: relay + TF (existing behaviour) ──────────
        # Topics are remapped from the camera's flat internal convention
        # to the standard hierarchical convention so downstream consumers
        # (recorder, TF) see identical topic names regardless of transport.
        actions.append(
            LogInfo(msg=f"{cam_name}: driver=dds — use realsense_poe_relay.launch.py")
        )
        # Publish static TF even for DDS cameras
        pos = cam_cfg.get("position", {}) or {}
        ori = cam_cfg.get("orientation", {}) or {}
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"{cam_name}_map_tf",
                output="screen",
                arguments=[
                    "--frame-id",
                    world_frame,
                    "--child-frame-id",
                    frame,
                    "--x",
                    str(pos.get("x", 0)),
                    "--y",
                    str(pos.get("y", 0)),
                    "--z",
                    str(pos.get("z", 0)),
                    "--roll",
                    str(ori.get("roll", 0)),
                    "--pitch",
                    str(ori.get("pitch", 0)),
                    "--yaw",
                    str(ori.get("yaw", 0)),
                ],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=_config_path(),
                description="Path to realsense_config.yaml",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
