"""Minimal ROS 2 stand-ins for the off-rig tests (not a test module).

``recording_manager`` and ``colour_video_recorder`` import rclpy/sensor_msgs, so
they cannot be imported on a workstation without ROS. ``ensure_ros_available()``
installs just enough of those modules to import the real code and drive its real
methods against stub-free helpers (subprocesses, files, threads) — no node is
ever created and no DDS/ROS graph is touched.

On a machine with a real ROS 2 install the stubs are not installed at all.
"""

import sys
import types
from importlib.util import find_spec


def ensure_ros_available():
    """Install the stubs only when the real ROS 2 python stack is missing."""
    if "rclpy" in sys.modules:
        return
    if find_spec("rclpy") is not None:
        return
    _install()


def _install():
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *args, **kwargs: None
    rclpy.ok = lambda: True
    rclpy.shutdown = lambda *args, **kwargs: None
    rclpy.spin = lambda *args, **kwargs: None
    rclpy.spin_once = lambda *args, **kwargs: None

    node_module = types.ModuleType("rclpy.node")

    class Node:
        def __init__(self, *args, **kwargs):
            del args, kwargs

    node_module.Node = Node

    parameter_module = types.ModuleType("rclpy.parameter")

    class Parameter:
        class Type:
            STRING_ARRAY = "string_array"
            BOOL = "bool"
            DOUBLE = "double"
            INTEGER = "integer"
            STRING = "string"

    parameter_module.Parameter = Parameter

    qos_module = types.ModuleType("rclpy.qos")

    class QoSProfile:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    class QoSReliabilityPolicy:
        BEST_EFFORT = "best_effort"
        RELIABLE = "reliable"

    class QoSHistoryPolicy:
        KEEP_LAST = "keep_last"
        KEEP_ALL = "keep_all"

    qos_module.QoSProfile = QoSProfile
    qos_module.QoSReliabilityPolicy = QoSReliabilityPolicy
    qos_module.QoSHistoryPolicy = QoSHistoryPolicy

    srv_module = types.ModuleType("std_srvs.srv")

    class Trigger:
        pass

    srv_module.Trigger = Trigger
    std_srvs = types.ModuleType("std_srvs")

    msg_module = types.ModuleType("sensor_msgs.msg")

    class Image:
        pass

    class CompressedImage:
        pass

    msg_module.Image = Image
    msg_module.CompressedImage = CompressedImage
    sensor_msgs = types.ModuleType("sensor_msgs")

    rclpy.node = node_module
    rclpy.parameter = parameter_module
    rclpy.qos = qos_module
    std_srvs.srv = srv_module
    sensor_msgs.msg = msg_module

    sys.modules.update(
        {
            "rclpy": rclpy,
            "rclpy.node": node_module,
            "rclpy.parameter": parameter_module,
            "rclpy.qos": qos_module,
            "std_srvs": std_srvs,
            "std_srvs.srv": srv_module,
            "sensor_msgs": sensor_msgs,
            "sensor_msgs.msg": msg_module,
        }
    )
