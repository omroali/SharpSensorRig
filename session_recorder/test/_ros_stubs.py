"""Minimal ROS 2 stand-ins for the off-rig tests (not a test module).

``recording_manager`` and ``colour_video_recorder`` import rclpy/sensor_msgs, so
they cannot be imported on a workstation without ROS. ``ensure_ros_available()``
installs just enough of those modules to import the real code and drive its real
methods against stub-free helpers (subprocesses, files, threads) — no node is
ever created and no DDS/ROS graph is touched.

``topic_health_monitor`` additionally imports fastapi/uvicorn (installed in the
container via python3-fastapi / python3-uvicorn, absent on a workstation), so
minimal versions of those are stubbed too. The web app is never built here; the
tests drive the monitor's plain methods directly.

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

    # rclpy.qos exposes the enum both under its own name and as ReliabilityPolicy
    # (the name topic_health_monitor imports).
    ReliabilityPolicy = QoSReliabilityPolicy

    class QoSHistoryPolicy:
        KEEP_LAST = "keep_last"
        KEEP_ALL = "keep_all"

    qos_module.QoSProfile = QoSProfile
    qos_module.QoSReliabilityPolicy = QoSReliabilityPolicy
    qos_module.QoSHistoryPolicy = QoSHistoryPolicy
    qos_module.ReliabilityPolicy = ReliabilityPolicy

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

    # topic_health_monitor builds a throwaway node + executor for its
    # start/stop service calls; those imports must resolve off-rig even though
    # no client is ever created here.
    callback_groups = types.ModuleType("rclpy.callback_groups")

    class ReentrantCallbackGroup:
        pass

    callback_groups.ReentrantCallbackGroup = ReentrantCallbackGroup

    executors = types.ModuleType("rclpy.executors")

    class SingleThreadedExecutor:
        def __init__(self, *args, **kwargs):
            pass

        def add_node(self, *args, **kwargs):
            pass

        def remove_node(self, *args, **kwargs):
            pass

        def spin_once(self, *args, **kwargs):
            pass

    executors.SingleThreadedExecutor = SingleThreadedExecutor

    # Used only to resolve a topic's message class for a raw-byte subscription.
    rosidl_utils = types.ModuleType("rosidl_runtime_py.utilities")

    def get_message(type_name):
        return object

    rosidl_utils.get_message = get_message
    rosidl_runtime_py = types.ModuleType("rosidl_runtime_py")
    rosidl_runtime_py.utilities = rosidl_utils

    sys.modules.update(
        {
            "rclpy": rclpy,
            "rclpy.node": node_module,
            "rclpy.parameter": parameter_module,
            "rclpy.qos": qos_module,
            "rclpy.callback_groups": callback_groups,
            "rclpy.executors": executors,
            "rosidl_runtime_py": rosidl_runtime_py,
            "rosidl_runtime_py.utilities": rosidl_utils,
            "std_srvs": std_srvs,
            "std_srvs.srv": srv_module,
            "sensor_msgs": sensor_msgs,
            "sensor_msgs.msg": msg_module,
        }
    )

    _install_web_stubs()


def _install_web_stubs():
    """Stand in for fastapi/uvicorn, which only exist in the rig container.

    Enough to import :mod:`session_recorder.topic_health_monitor`. The decorators
    return the function unchanged so nothing is silently swallowed, and ``app``
    construction is never exercised off-rig.
    """
    if find_spec("fastapi") is None:
        fastapi = types.ModuleType("fastapi")

        class FastAPI:  # noqa: D401 - stand-in
            def __init__(self, *args, **kwargs):
                self.routes = {}

            def _register(self, path):
                def decorator(func):
                    self.routes[path] = func
                    return func
                return decorator

            def get(self, path, **kwargs):
                return self._register(path)

            def post(self, path, **kwargs):
                return self._register(path)

        fastapi.FastAPI = FastAPI
        responses = types.ModuleType("fastapi.responses")

        class HTMLResponse:
            def __init__(self, content=""):
                self.body = content

        class JSONResponse:
            def __init__(self, content=None, status_code=200):
                self.body = content if content is not None else {}
                self.status_code = status_code

        responses.HTMLResponse = HTMLResponse
        responses.JSONResponse = JSONResponse
        sys.modules.update({
            "fastapi": fastapi,
            "fastapi.responses": responses,
        })

    if find_spec("uvicorn") is None:
        uvicorn = types.ModuleType("uvicorn")

        class Config:
            def __init__(self, *args, **kwargs):
                self.args = args
                self.kwargs = kwargs

        class Server:
            def __init__(self, config=None):
                self.config = config
                self.should_exit = False

            def run(self):
                return None

        uvicorn.Config = Config
        uvicorn.Server = Server
        sys.modules["uvicorn"] = uvicorn
