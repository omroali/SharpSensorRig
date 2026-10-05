"""Unit tests for the recording-safety helpers (session_recorder.hardening).

Pure-function tests: no ROS runtime, no sensors, no GPU. Run with:
  python3 -m pytest session_recorder/test/test_hardening.py

Covers the pieces of the 2026-10-01 hardening work that can be proven off-rig:
bag splitting arguments, the SIGHUP/SIGTERM stop-request wiring (real signals in
a subprocess), watchdog verdicts, and the session status files.
"""

import json
import os
import re
import signal
import subprocess
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from session_recorder import hardening  # noqa: E402

PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


# --- bag splitting (A1) -----------------------------------------------------
def test_bag_split_args_convert_mib_to_bytes_and_keep_seconds():
    args = hardening.bag_split_args(1024, 300)
    assert args == [
        "--max-bag-size", str(1024 * 1024 * 1024), "--max-bag-duration", "300"
    ]


def test_bag_split_args_omit_disabled_limits():
    """0/None must reproduce the old one-unbounded-bag behaviour."""
    assert hardening.bag_split_args(0, 0) == []
    assert hardening.bag_split_args(None, None) == []
    assert hardening.bag_split_args(0, 120) == ["--max-bag-duration", "120"]
    assert hardening.bag_split_args(512, 0) == [
        "--max-bag-size", str(512 * 1024 * 1024)
    ]


def test_bag_split_args_truncate_fractional_values():
    assert hardening.bag_split_args(1.5, 90.7) == [
        "--max-bag-size", str(1024 * 1024 * 3 // 2), "--max-bag-duration", "90",
    ]


# --- video health (B3) ------------------------------------------------------
def test_video_startup_problems_names_missing_and_empty_files(tmp_path):
    videos = tmp_path / "videos"
    videos.mkdir()
    (videos / "cam_color_image_raw.mp4").write_bytes(b"")  # 0 bytes: the incident
    (videos / "cam_ir_image.mp4").write_bytes(b"\x00" * 128)  # healthy

    problems = hardening.video_startup_problems(
        str(tmp_path), ["/cam/color/image_raw", "/cam/ir/image", "/cam/other"]
    )
    assert len(problems) == 2
    assert any("cam_color_image_raw.mp4" in p and "0 bytes" in p for p in problems)
    assert any("no video file yet" in p for p in problems)
    assert not any("cam_ir_image.mp4" in p for p in problems)


def test_video_file_sizes_use_minus_one_for_uncreated_files(tmp_path):
    videos = tmp_path / "videos"
    videos.mkdir()
    (videos / "a.mp4").write_bytes(b"x" * 10)
    sizes = hardening.video_file_sizes(str(tmp_path), ["/a", "/b"])
    assert sizes == {"/a": 10, "/b": -1}
    assert hardening.total_size([str(videos / "a.mp4"), str(videos / "b.mp4")]) == 10


def test_bag_file_sizes_are_relative_to_the_session(tmp_path):
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "bag_0.mcap").write_bytes(b"x" * 5)
    (bag / "bag_1.mcap").write_bytes(b"x" * 7)
    assert hardening.bag_file_sizes(str(tmp_path)) == {
        os.path.join("bag", "bag_0.mcap"): 5,
        os.path.join("bag", "bag_1.mcap"): 7,
    }


# --- session status files ---------------------------------------------------
def test_write_atomic_json_roundtrip_leaves_no_tmp_file(tmp_path):
    path = tmp_path / "sub" / "status.json"
    hardening.write_atomic_json(path, {"b": 2, "a": 1})
    assert json.loads(path.read_text()) == {"a": 1, "b": 2}
    assert [p.name for p in path.parent.iterdir()] == ["status.json"]


def test_read_json_never_raises_on_missing_or_corrupt_files(tmp_path):
    assert hardening.read_json(str(tmp_path / "absent.json")) == {}
    broken = tmp_path / "broken.json"
    broken.write_text("{not json")
    assert hardening.read_json(str(broken)) == {}
    scalar = tmp_path / "scalar.json"
    scalar.write_text("3")
    assert hardening.read_json(str(scalar)) == {}


def test_encoder_status_and_stream_failures_share_the_videos_dir(tmp_path):
    assert hardening.encoder_status_path(str(tmp_path)) == os.path.join(
        str(tmp_path), "videos", hardening.ENCODER_STATUS_NAME
    )
    hardening.write_encoder_status(str(tmp_path), {"fallback_used": True})
    assert hardening.read_encoder_status(str(tmp_path))["fallback_used"] is True

    hardening.write_stream_failures(
        str(tmp_path), {"/cam/color": {"reason": "ffmpeg exited early"}}
    )
    failures = hardening.read_stream_failures(str(tmp_path))
    assert failures["/cam/color"]["reason"] == "ffmpeg exited early"
    assert hardening.new_failure_keys([], failures) == ["/cam/color"]
    assert hardening.new_failure_keys(["/cam/color"], failures) == []


def test_append_event_log_is_timestamped_and_appended(tmp_path):
    path = tmp_path / "watchdog.log"
    hardening.append_event_log(path, "ALARM one")
    hardening.append_event_log(path, "NOTE two")
    lines = path.read_text().splitlines()
    assert len(lines) == 2
    assert lines[0].endswith("ALARM one") and lines[1].endswith("NOTE two")
    assert lines[0][:4].isdigit()  # ISO timestamp


def test_bag_metadata_files_sort_numerically(tmp_path):
    for name in ("bag_10", "bag", "bag_2", "bag_1"):
        directory = tmp_path / name
        directory.mkdir()
        (directory / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
    names = [
        os.path.basename(os.path.dirname(path))
        for path in hardening.bag_metadata_files(str(tmp_path))
    ]
    assert names == ["bag", "bag_1", "bag_2", "bag_10"]


# --- watchdog verdicts (A4) -------------------------------------------------
class _FakeProc:
    def __init__(self, pid, returncode=None):
        self.pid = pid
        self.returncode = returncode

    def poll(self):
        return self.returncode


def test_poll_children_separates_alive_from_exited():
    statuses = hardening.poll_children(
        {
            "colour_video_recorder": _FakeProc(11, None),
            "rosbag_record": _FakeProc(22, 1),
            "topic_throttle": None,
        }
    )
    by_name = {status.name: status for status in statuses}
    assert set(by_name) == {"colour_video_recorder", "rosbag_record"}
    assert by_name["colour_video_recorder"].alive
    assert not by_name["rosbag_record"].alive
    assert by_name["rosbag_record"].returncode == 1
    assert "exited with code 1" in by_name["rosbag_record"].summary()


def test_dead_children_only_lists_the_dead_ones():
    dead = hardening.dead_children(
        {"a": _FakeProc(1, None), "b": _FakeProc(2, -9), "c": _FakeProc(3, 0)}
    )
    assert [status.name for status in dead] == ["b", "c"]


def test_build_heartbeat_is_flat_json_friendly():
    statuses = hardening.poll_children({"rosbag_record": _FakeProc(22, 1)})
    payload = hardening.build_heartbeat(
        session_dir="/data/session_1",
        participant="uuid-1",
        active=True,
        started_at=1000.0,
        children={status.name: status for status in statuses},
        bag_restarts=1,
        watchdog_events=["rosbag_record exited"],
        video_sizes={"/cam/color": 4096},
        bag_sizes={"bag/bag_0.mcap": 8192},
        now=1060.0,
    )
    assert payload["active"] is True
    assert payload["elapsed_s"] == 60.0
    assert payload["children"]["rosbag_record"]["returncode"] == 1
    assert payload["bag_restarts"] == 1
    assert payload["video_bytes"] == {"/cam/color": 4096}
    assert json.loads(json.dumps(payload)) == payload  # serialisable as-is


# --- signal wiring (A2) -----------------------------------------------------
_SIGNAL_PROBE = """
import sys, time, signal
sys.path.insert(0, {package!r})
from session_recorder import hardening

request = hardening.StopRequest()
installed = hardening.install_stop_handlers(request)
print("INSTALLED", len(installed))
signal.raise_signal(getattr(signal, sys.argv[1]))
deadline = time.time() + 2.0
while not request.requested and time.time() < deadline:
    time.sleep(0.01)
print("REQUESTED", request.requested, "|", request.reason())
sys.exit(0 if request.requested else 3)
"""


@pytest.mark.parametrize("signame", ["SIGTERM", "SIGHUP"])
def test_stop_handlers_turn_a_real_signal_into_a_stop_request(signame):
    """The graceful stop must be reachable from a signal, not only Ctrl-C."""
    proc = subprocess.run(
        [sys.executable, "-c", _SIGNAL_PROBE.format(package=PACKAGE_ROOT), signame],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert proc.returncode == 0, proc.stderr
    assert "INSTALLED 2" in proc.stdout
    assert f"REQUESTED True | {signame} received" in proc.stdout


def test_stop_request_reason_and_clear():
    request = hardening.StopRequest()
    assert not request.requested
    assert request.reason() == "stop requested"
    request.request(signal.SIGTERM)
    assert request.requested
    assert request.reason() == "SIGTERM received"
    request.clear()
    assert not request.requested and request.reason() == "stop requested"


def test_signal_name_never_raises():
    assert hardening.signal_name(signal.SIGHUP) == "SIGHUP"
    assert hardening.signal_name(9999) == "signal 9999"
    assert hardening.signal_name(None) == "unknown"


def test_restore_handlers_puts_the_previous_handler_back():
    request = hardening.StopRequest()
    previous = signal.getsignal(signal.SIGHUP)
    installed = hardening.install_stop_handlers(request, signals=[signal.SIGHUP])
    assert installed and installed[0][0] == signal.SIGHUP
    hardening.restore_handlers(installed)
    assert signal.getsignal(signal.SIGHUP) is previous


# --- static wiring checks ---------------------------------------------------
# recording_manager imports rclpy, so its signal wiring cannot be imported here;
# the behavioural half of this contract is the subprocess test above.
def _source(name):
    with open(os.path.join(PACKAGE_ROOT, "session_recorder", name), encoding="utf-8") as handle:
        return handle.read()


def test_manager_installs_handlers_and_runs_the_graceful_stop():
    source = _source("recording_manager.py")
    assert "install_stop_handlers(stop)" in source
    assert "node.graceful_shutdown(reason)" in source
    assert "bag_split_args(self._bag_max_size_mb, self._bag_max_duration_s)" in source


def test_video_recorder_installs_handlers_too():
    """SIGTERM/SIGHUP must finalise the mp4 instead of truncating it."""
    source = _source("colour_video_recorder.py")
    assert "install_stop_handlers(stop)" in source


def test_launch_file_passes_every_manager_parameter():
    """A new parameter the launch file forgets would be silently ignored."""
    declared = set(
        re.findall(r'declare_parameter\("([^"]+)"', _source("recording_manager.py"))
    )
    with open(
        os.path.join(PACKAGE_ROOT, "launch", "unified_recording.launch.py"),
        encoding="utf-8",
    ) as handle:
        launch_source = handle.read()
    passed = set(re.findall(r'"([a-z_][a-z0-9_]*)":', launch_source))
    assert sorted(declared - passed) == []


def test_color_command_only_passes_declared_recorder_parameters():
    manager_source = _source("recording_manager.py")
    color_cmd_body = manager_source.split("def _build_color_cmd", 1)[1].split(
        "def _build_bag_cmd", 1
    )[0]
    passed = set(re.findall(r'f"([a-z_]+):=', color_cmd_body))
    declared = set(
        re.findall(r'declare_parameter\("([^"]+)"', _source("colour_video_recorder.py"))
    )
    assert passed <= declared, sorted(passed - declared)
    assert "software_fallback" in passed  # the encoder fallback must be plumbed

