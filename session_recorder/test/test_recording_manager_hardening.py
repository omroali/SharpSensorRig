"""Off-rig tests for the recording manager's hardening behaviour.

`recording_manager` imports rclpy, so it cannot be imported on a workstation
without ROS. These tests install the minimum ROS stubs needed to import the
module and then call the *real* methods on an instance built with ``__new__``
(no node, no ROS graph, no sensors):

* ``_build_bag_cmd``  -> splitting flags really are in the command (A1)
* ``_build_color_cmd``-> the software-fallback parameter reaches the recorder (B8)
* ``_stop_processes`` -> SIGINT is forwarded to the ``os.setsid`` children (A2)
* ``_watchdog_tick``  -> a dead recorder is noticed, logged, restarted, and the
  heartbeat + 0-byte-video alarm are written (A4/B3)

Run with:
  python3 -m pytest session_recorder/test/test_recording_manager_hardening.py
"""

import importlib
import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

from _ros_stubs import ensure_ros_available  # noqa: E402


class NullLogger:
    def __init__(self):
        self.messages = []

    def _record(self, level, message):
        self.messages.append((level, str(message)))

    def debug(self, message):
        self._record("debug", message)

    def info(self, message):
        self._record("info", message)

    def warn(self, message):
        self._record("warn", message)

    def error(self, message):
        self._record("error", message)

    def errors(self):
        return [message for level, message in self.messages if level == "error"]


def _load_manager_module():
    ensure_ros_available()
    return importlib.import_module("session_recorder.recording_manager")


def _manager(module):
    """A RecordingManager with no ROS node behind it, set up for the tests."""
    manager = module.RecordingManager.__new__(module.RecordingManager)
    manager._lock = threading.Lock()
    manager._active = False
    manager._session_dir = ""
    manager._session_start = 0.0
    manager._color_proc = None
    manager._bag_proc = None
    manager._throttle_proc = None
    manager._color_log = None
    manager._bag_log = None
    manager._throttle_log = None
    manager._stop_grace_s = 5.0
    manager._watchdog_interval_s = 5.0
    manager._video_startup_grace_s = 20.0
    manager._watchdog_restart_bag = True
    manager._watchdog_restart_throttle = True
    manager._nexus_capture = False
    manager._participant_id = "test-participant"
    manager._expected_video_topics = []
    manager._test_logger = NullLogger()
    manager.get_logger = lambda: manager._test_logger
    manager._reset_watchdog_state()
    return manager


def test_bag_command_splits_the_bag():
    """A1: one crash must cost one split, not the whole 72-minute session."""
    module = _load_manager_module()
    manager = _manager(module)
    manager._bag_storage = "mcap"
    manager._bag_cache_size_mb = 128
    manager._bag_compression = "storage"
    manager._mcap_preset = "zstd_fast"
    manager._bag_regex = "/vicon/.*"
    manager._bag_exclude_regex = ""
    manager._bag_topics = ["/tf", "/tf_static"]
    manager._bag_max_size_mb = 1024.0
    manager._bag_max_duration_s = 300.0

    cmd = manager._build_bag_cmd("/data/participant/session_x/bag")
    assert cmd[:4] == ["ros2", "bag", "record", "-o"]
    assert cmd[cmd.index("--max-bag-size") + 1] == str(1024 * 1024 * 1024)
    assert cmd[cmd.index("--max-bag-duration") + 1] == "300"
    # the pre-existing options are untouched
    assert cmd[cmd.index("--storage") + 1] == "mcap"
    assert cmd[cmd.index("--storage-preset-profile") + 1] == "zstd_fast"
    assert cmd[cmd.index("--topics") + 1 :] == ["/tf", "/tf_static"]


def test_bag_command_without_limits_stays_backwards_compatible():
    module = _load_manager_module()
    manager = _manager(module)
    manager._bag_storage = "sqlite3"
    manager._bag_cache_size_mb = 64
    manager._bag_compression = "none"
    manager._mcap_preset = "zstd_fast"
    manager._bag_regex = ""
    manager._bag_exclude_regex = ""
    manager._bag_topics = []
    manager._bag_max_size_mb = 0.0
    manager._bag_max_duration_s = 0.0

    cmd = manager._build_bag_cmd("/data/bag")
    assert "--max-bag-size" not in cmd
    assert "--max-bag-duration" not in cmd


def test_color_command_passes_the_software_fallback_flag():
    module = _load_manager_module()
    manager = _manager(module)
    manager._color_topics = ["/cam/color/image_raw"]
    manager._color_compressed_topics = []
    manager._topic_fps_overrides = []
    manager._color_remaps = ["/a/color:=/b/color"]
    manager._video_encoder = "hevc_nvenc"
    manager._video_software_fallback = True
    manager._video_crf = 18
    manager._video_fps = 30.0

    cmd = manager._build_color_cmd("/data/session/videos")
    assert "software_fallback:=true" in cmd
    assert "encoder:=hevc_nvenc" in cmd
    assert "-r" in cmd and "/a/color:=/b/color" in cmd

    manager._video_software_fallback = False
    assert "software_fallback:=false" in manager._build_color_cmd("/tmp")


def test_stop_processes_forwards_sigint_to_the_isolated_child(tmp_path):
    """A2: children are in their own session, so the manager must forward."""
    module = _load_manager_module()
    manager = _manager(module)
    manager._session_dir = str(tmp_path)
    manager._active = True

    proc, handle = manager._start_process(
        ["sleep", "60"], str(tmp_path / "child.log")
    )
    manager._color_proc = proc
    manager._color_log = handle
    assert os.getpgid(proc.pid) != os.getpgid(0)  # preexec_fn=os.setsid

    session_dir = manager._stop_processes()
    assert session_dir == str(tmp_path)
    assert proc.poll() is not None  # SIGINT was forwarded and it exited
    assert manager._active is False
    assert manager._color_proc is None
    assert handle.closed


def test_watchdog_detects_a_dead_recorder_and_restarts_the_bag(tmp_path):
    module = _load_manager_module()
    manager = _manager(module)
    session = tmp_path / "session_20261006_090000"
    session.mkdir()
    manager._session_dir = str(session)
    manager._session_start = time.time() - 120
    manager._active = True
    # Keep the restart command host-runnable: the point under test is the
    # detection/restart path, not ros2 itself.
    manager._build_bag_cmd = lambda bag_dir: ["sleep", "60"]

    bag_log = session / "rosbag_record.log"
    dead_proc, dead_log = manager._start_process(["sleep", "0.05"], str(bag_log))
    dead_proc.wait(timeout=5)
    manager._bag_proc = dead_proc
    manager._bag_log = dead_log

    manager._watchdog_tick()

    errors = manager._test_logger.errors()
    assert any("rosbag_record" in message for message in errors)
    assert any("restart" in message.lower() for message in errors)
    assert manager._bag_proc is not None
    assert manager._bag_proc.poll() is None  # the replacement is alive
    assert manager._bag_restarts == 1

    log_lines = (session / "watchdog.log").read_text().splitlines()
    assert any("restarted the bag recorder" in line for line in log_lines)

    heartbeat = json.loads((session / "recorder_heartbeat.json").read_text())
    assert heartbeat["active"] is True
    assert heartbeat["bag_restarts"] == 1
    assert heartbeat["children"]["rosbag_record"]["alive"] is True
    assert heartbeat["watchdog_events"]

    # The death is alarmed once per episode, not once per tick.
    before = len([m for m in errors if "rosbag_record" in m])
    manager._watchdog_tick()
    after = len([m for m in manager._test_logger.errors() if "rosbag_record" in m])
    assert after == before

    manager._stop_processes()


def test_watchdog_alarms_on_a_zero_byte_video(tmp_path):
    """B3: the 2026-10-01 failure mode must reach the operator's console."""
    module = _load_manager_module()
    manager = _manager(module)
    session = tmp_path / "session_20261006_090000"
    (session / "videos").mkdir(parents=True)
    zero_byte = session / "videos" / "cam_color_image_raw.mp4"
    zero_byte.write_bytes(b"")
    manager._session_dir = str(session)
    manager._session_start = time.time() - 30
    manager._video_startup_grace_s = 0.0
    manager._active = True
    manager._expected_video_topics = ["/cam/color/image_raw"]

    manager._watchdog_tick()

    errors = manager._test_logger.errors()
    assert any("VIDEO CHECK FAILED" in message and "0 bytes" in message for message in errors)
    assert any("VIDEO CHECK FAILED" in line for line in (session / "watchdog.log").read_text().splitlines())


def test_watchdog_surfaces_recorder_stream_failures_once(tmp_path):
    """The recorder writes stream_failures.json; the manager must echo it."""
    module = _load_manager_module()
    from session_recorder import hardening

    manager = _manager(module)
    session = tmp_path / "session_20261006_090000"
    (session / "videos").mkdir(parents=True)
    manager._session_dir = str(session)
    manager._session_start = time.time() - 60
    manager._active = True

    hardening.write_stream_failures(
        str(session),
        {
            "/cam/color/image_raw": {
                "topic": "/cam/color/image_raw",
                "reason": "ffmpeg exited early (encoder=h264_nvenc) after 0 frames",
                "severity": "error",
                "frames": 0,
            }
        },
    )

    manager._watchdog_tick()
    first = manager._test_logger.errors()
    assert any("STREAM FAILURE [/cam/color/image_raw]" in message for message in first)

    manager._watchdog_tick()
    assert manager._test_logger.errors() == first  # not re-alarmed every tick


def test_graceful_shutdown_runs_the_full_stop_path(tmp_path):
    module = _load_manager_module()
    manager = _manager(module)
    session = tmp_path / "session_20261006_090000"
    session.mkdir()
    manager._session_dir = str(session)
    manager._active = True
    proc, handle = manager._start_process(["sleep", "60"], str(session / "bag.log"))
    manager._bag_proc = proc
    manager._bag_log = handle
    checked = []

    def fake_check(session_dir):
        checked.append(session_dir)
        return "session_check: PASS (1/1 checks ok, 0 warnings)"

    manager._run_session_check = fake_check

    session_dir, summary = manager.graceful_shutdown("SIGHUP received")
    assert session_dir == str(session)
    assert summary.startswith("session_check: PASS")
    assert checked == [str(session)]
    assert proc.poll() is not None
    assert manager._active is False

    # Idempotent: a second call (e.g. destroy_node after main's finally) is a no-op.
    assert manager.graceful_shutdown("again") == ("", "")


def test_session_check_enabled_flag_gates_the_report_writer():
    """_run_session_check must not shell out when the check is disabled."""
    module = _load_manager_module()
    manager = _manager(module)
    manager._session_check_enabled = False
    assert manager._run_session_check("/tmp/whatever") == ""


def test_watchdog_does_nothing_while_inactive(tmp_path):
    module = _load_manager_module()
    manager = _manager(module)
    manager._session_dir = str(tmp_path)
    manager._active = False
    manager._watchdog_tick()  # must not raise or write anything
    assert not (tmp_path / "recorder_heartbeat.json").exists()


def test_start_process_isolates_children_in_their_own_session(tmp_path):
    module = _load_manager_module()
    manager = _manager(module)
    proc, handle = manager._start_process(["sleep", "5"], str(tmp_path / "x.log"))
    try:
        assert os.getpgid(proc.pid) != os.getpgid(0)
        assert proc.poll() is None
    finally:
        os.killpg(os.getpgid(proc.pid), 9)
        proc.wait(timeout=5)
        handle.close()
