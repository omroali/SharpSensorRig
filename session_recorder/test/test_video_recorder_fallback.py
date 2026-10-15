"""Off-rig tests for the video recorder's encoder fallback and retry.

The 2026-10-01 failure was: ffmpeg could not initialise NVENC, every stream died
before writing a frame, and the four 0-byte files went unnoticed. These tests
drive the *real* StreamRecorder class (with ROS stubbed out and a fake camera
message) through:

* encoder selection when the NVENC probe fails -> libx264, no retry needed;
* ffmpeg dying before the first frame -> one retry with libx264, first failure
  log preserved, subsequent frames written;
* a stream that ends with a 0-byte file -> an explicit VIDEO LOSS record that
  the manager turns into a console alarm.

Run with:
  python3 -m pytest session_recorder/test/test_video_recorder_fallback.py
"""

import os
import subprocess
import sys
import time
import types

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

from _ros_stubs import ensure_ros_available  # noqa: E402

ensure_ros_available()

from session_recorder import colour_video_recorder as cvi  # noqa: E402
from session_recorder import video_encoding as ve  # noqa: E402

TOPIC = "/cam/color/image_raw"
STEM = "cam_color_image_raw"


class FakeLogger:
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

    def text(self):
        return "\n".join(message for _, message in self.messages)


class FakeImage:
    """The parts of sensor_msgs/Image the recorder actually reads."""

    def __init__(self, ts_ns, width=8, height=8, encoding="bgr8"):
        self.header = types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=ts_ns // 1_000_000_000, nanosec=ts_ns % 1_000_000_000)
        )
        self.width = width
        self.height = height
        self.encoding = encoding
        self.step = width * 3
        self.data = bytes(width * height * 3)


def _probe(results):
    def run(cmd, capture_output=True, text=True, timeout=None):
        del capture_output, text, timeout
        encoder = cmd[cmd.index("-c:v") + 1]
        returncode, stderr = results.get(encoder, (1, f"Unknown encoder '{encoder}'"))
        return subprocess.CompletedProcess(cmd, returncode, "", stderr)

    return run


def _recorder(tmp_path, results, fps=30.0):
    logger = FakeLogger()
    resolver = ve.EncoderResolver(
        requested="hevc_nvenc", probe=lambda e: ve.probe_encoder(e, run=_probe(results))
    )
    failures = cvi.StreamFailureLog(str(tmp_path), logger)
    recorder = cvi.StreamRecorder(
        TOPIC, str(tmp_path), 18, "hevc_nvenc", fps, 5, logger, resolver, failures
    )
    return recorder, logger, failures


def _feed(recorder, frames=21, step_ns=100_000_000):
    for index in range(frames):
        recorder.write(FakeImage(index * step_ns))


def _until(predicate, timeout=10.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_nvenc_probe_failure_selects_libx264_without_a_retry(tmp_path):
    recorder, logger, _ = _recorder(
        tmp_path, {"h264_nvenc": (1, "cuInit(0) failed -> CUDA_ERROR_NO_DEVICE"), "libx264": (0, "")}
    )
    try:
        assert recorder._encoder == "libx264"
        assert recorder._requested_encoder == "h264_nvenc"
        assert recorder._retry_encoder is None  # nothing left to retry with
    finally:
        recorder.stop()


def test_ffmpeg_death_before_the_first_frame_retries_with_the_software_encoder(tmp_path):
    recorder, logger, failures = _recorder(
        tmp_path, {"h264_nvenc": (0, ""), "libx264": (0, "")}
    )
    assert recorder._encoder == "h264_nvenc"
    assert recorder._retry_encoder == "libx264"

    spawned = []

    def fake_spawn():
        spawned.append(recorder._encoder)
        # The real _spawn_ffmpeg opens <stem>.ffmpeg.log (truncating) first.
        with open(recorder._log_path, "w", encoding="utf-8") as handle:
            handle.write(f"fake ffmpeg stderr for {recorder._encoder}\n")
        if len(spawned) == 1:
            # Models an NVENC init failure that the probe did not catch: the
            # process is already dead before the first frame is handed over.
            proc = subprocess.Popen(
                ["sh", "-c", "exit 1"], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL
            )
            proc.wait(timeout=5)
            recorder._proc = proc
        else:
            recorder._proc = subprocess.Popen(
                ["cat"], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL
            )

    recorder._spawn_ffmpeg = fake_spawn
    try:
        _feed(recorder)
        assert _until(lambda: recorder._retried), "the retry never ran"
        assert spawned == ["h264_nvenc", "libx264"]
        assert recorder._encoder == "libx264"
        assert "retried once with libx264" in logger.text()
        # The first failure's ffmpeg log is preserved for forensics.
        assert os.path.exists(str(tmp_path / f"{STEM}.ffmpeg.log.attempt1"))
        # The fallback is also visible to the manager's console alarm channel.
        assert failures.failures[TOPIC]["severity"] == "warn"
        assert _until(lambda: recorder.frame_idx > 0), "no frames reached the retry"
    finally:
        recorder.stop()


def test_a_zero_byte_video_is_recorded_as_a_loss(tmp_path):
    """The exact incident signature: ffmpeg never wrote anything."""
    recorder, logger, failures = _recorder(
        tmp_path, {"h264_nvenc": (1, "cuInit(0) failed"), "libx264": (1, "no libx264")}
    )
    try:
        recorder.stop()
    finally:
        pass
    assert "VIDEO LOSS" in logger.text()
    assert failures.failures[TOPIC]["severity"] == "error"
    assert "0 bytes" in failures.failures[TOPIC]["reason"]


def test_stop_is_safe_when_ffmpeg_was_never_started(tmp_path):
    recorder, _, _ = _recorder(tmp_path, {"h264_nvenc": (0, "")})
    recorder.stop()  # no frames arrived; must not raise on a None _proc


# --- mux-rate calibration (2026-10-08: startup window baked in for the whole
# --- session, so video and its timestamp CSV drifted apart) ------------------

def _ramp_timestamps(nominal_fps=30.0, startup_gaps_s=(), total_s=3.5):
    """Timestamps for a stream that stalls at the start then runs at nominal.

    Reproduces what the rig's sensors actually do: the median inter-frame gap is
    nominal from the very first frame, but the opening seconds contain multi-frame
    gaps while the capture pipeline primes. Averaging over that window (what the
    recorder used to do, over its first 2 s) under-reports the rate, and the
    recorder then muxed the entire session at that wrong rate.
    """
    step_ns = int(1e9 / nominal_fps)
    stamps = [0]
    for gap_s in startup_gaps_s:
        stamps.append(stamps[-1] + step_ns + int(gap_s * 1e9))
    while stamps[-1] < int(total_s * 1e9):
        stamps.append(stamps[-1] + step_ns)
    return stamps


def test_sustained_fps_ignores_startup_stalls():
    """The measurement must reflect the settled rate, not the ramp-up."""
    stamps = _ramp_timestamps(startup_gaps_s=(0.035, 0.070, 0.098, 0.034, 0.067))
    naive = cvi.measured_fps(stamps)
    settled = cvi.sustained_fps(stamps, fallback_fps=30.0)
    assert naive < 29.0, f"fixture should look slow to the old measurement: {naive}"
    assert settled == pytest.approx(30.0, abs=0.5), (
        f"settled rate should recover ~30 fps, got {settled} (naive was {naive})"
    )


def test_sustained_fps_reports_a_genuinely_slow_stream():
    """A stream that is really slow must still be measured as slow.

    This is the behaviour the calibration exists for: a Kinect starved of USB
    bandwidth delivers ~24 fps throughout, and the video must not be stamped as
    if it arrived at 30.
    """
    stamps = _ramp_timestamps(nominal_fps=24.0, total_s=5.0)
    assert cvi.sustained_fps(stamps, fallback_fps=30.0) == pytest.approx(24.0, abs=0.6)


def test_sustained_fps_falls_back_when_the_window_is_too_short():
    """Fewer than two settled frames -> the caller's fallback, never a crash."""
    assert cvi.sustained_fps([], fallback_fps=30.0) == 30.0
    assert cvi.sustained_fps([0], fallback_fps=30.0) == 30.0
    # Two frames, both inside the settling period.
    assert cvi.sustained_fps([0, 33_000_000], fallback_fps=30.0) == 30.0


def test_sustained_fps_never_returns_zero_for_a_usable_window():
    stamps = _ramp_timestamps(total_s=6.0)
    assert cvi.sustained_fps(stamps, fallback_fps=30.0) > 0


def test_settle_window_is_shorter_than_the_calibration_wait():
    """The settle period must fit inside the window calibration can produce,
    otherwise every calibration would fall back and the feature would be dead."""
    assert cvi.CALIBRATION_SETTLE_S < cvi.CALIBRATION_MAX_WAIT_S
