#!/usr/bin/env python3
"""H.265/H.264 video recorder for camera colour/IR streams (any sensor).

Encoder policy (2026-10-01 incident)
------------------------------------
The encoder used to be hardcoded NVENC with no fallback, no retry and no
escalation. When the host's NVIDIA driver/library pair drifted, ffmpeg could not
initialise ``h264_nvenc``, every stream died within seconds of the start and all
four videos were left at 0 bytes — unnoticed until after the participant left.

This recorder now:

* probes the requested encoder with the same 0.1 s encode the operator
  pre-flight uses (:func:`session_recorder.video_encoding.probe_encoder`);
* falls back to libx264/libx265 when NVENC cannot initialise, and says so in the
  log and in ``videos/encoder_status.json`` (the manager echoes that onto the
  operator's console);
* retries a stream once with the software encoder if ffmpeg dies *before writing
  a single frame* (the incident's exact failure mode — the 0-byte file is
  overwritten, nothing is lost), keeping the first failure log as
  ``<stem>.ffmpeg.log.attempt1``;
* records every permanent stream loss in ``videos/stream_failures.json`` so the
  manager can turn it into a console alarm;
* stops cleanly on SIGTERM/SIGHUP, finalising the Matroska file instead of
  leaving its last cluster truncated.
"""

import csv
import os
import subprocess
import threading
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from session_recorder import hardening
from session_recorder.video_encoding import (
    EncoderResolver,
    build_encoder_args,
    build_muxer_args,
    keyframe_interval,
    software_fallback_for,
)


def topic_to_filename(topic: str) -> str:
    return topic.strip("/").replace("/", "_")


# ffmpeg is told a fixed mux rate (`-r`/`-framerate`) and has no visibility
# into real frame timing, so if the sensor can't sustain the configured fps
# (e.g. a Kinect starved of USB bandwidth) the encoded video plays back
# faster than real time - fewer real frames get stamped as if they arrived
# at the full configured rate. To avoid that, each recorder holds the first
# few seconds of (post-throttle) frames back, measures the rate they're
# actually arriving at, and muxes at whichever of {configured, measured} is
# lower - never faster than what the sensor is really delivering.
CALIBRATION_MIN_FRAMES = 10
CALIBRATION_WINDOW_S = 2.0
CALIBRATION_MAX_WAIT_S = 5.0

#: Sensors ramp up: a Kinect/QHD stream is slow for the first second or two
#: while its pipeline primes, then settles at its real rate. Measuring the
#: *startup* window therefore under-reports it permanently.
#:
#: Measured on this rig (session_20261008_123838, recovered frame timestamps):
#: the Kinect calibrated at 24.11 fps and muxed the whole session at that rate
#: while genuinely delivering 30.06 fps, so playback ran ~20% slow and the
#: video drifted out of step with its own timestamp CSV - which is what every
#: downstream join between frames and CSV rows relies on. The RealSense pair
#: calibrated at 13.98/8.23 fps against a true 14.99/15.00 fps.
#:
#: Frames before SETTLE_S are therefore excluded from the measurement.
CALIBRATION_SETTLE_S = 3.0


def measured_fps(timestamps_ns: list[int]) -> float:
    """Mean rate across ``timestamps_ns`` (0.0 if it has fewer than two)."""
    if len(timestamps_ns) < 2:
        return 0.0
    span_s = (timestamps_ns[-1] - timestamps_ns[0]) / 1e9
    return (len(timestamps_ns) - 1) / span_s if span_s > 0 else 0.0


def sustained_fps(
    timestamps_ns: list[int],
    settle_s: float = CALIBRATION_SETTLE_S,
    fallback_fps: float = 0.0,
) -> float:
    """Frame rate over the *settled* part of a calibration window.

    Returns ``fallback_fps`` when the window is too short to skip the settling
    period and still have two frames left to measure between.
    """
    if len(timestamps_ns) < 2:
        return fallback_fps
    start_ns = timestamps_ns[0] + int(settle_s * 1e9)
    settled = [ts for ts in timestamps_ns if ts >= start_ns]
    if len(settled) < 2:
        return fallback_fps
    rate = measured_fps(settled)
    return rate if rate > 0 else fallback_fps


def calibration_done(timestamps_ns: list[int]) -> bool:
    if len(timestamps_ns) < 2:
        return False
    elapsed = (timestamps_ns[-1] - timestamps_ns[0]) / 1e9
    if len(timestamps_ns) >= CALIBRATION_MIN_FRAMES and elapsed >= CALIBRATION_WINDOW_S:
        return True
    return elapsed >= CALIBRATION_MAX_WAIT_S


def file_size(path: str) -> int:
    try:
        return os.path.getsize(path)
    except OSError:
        return 0


def usable_fallback(
    resolver: EncoderResolver, requested: str, effective: str
) -> str | None:
    """The software encoder to retry a stream with, if the probe says it works.

    Probes are cached per encoder, so this costs nothing when the fallback was
    already probed as part of resolving the stream.
    """
    fallback = software_fallback_for(requested)
    if not fallback or not resolver.allow_software_fallback or fallback == effective:
        return None
    return fallback if resolver.probe_result(fallback).ok else None


class StreamFailureLog:
    """Per-stream failure record written where the manager can see it.

    The manager polls ``videos/stream_failures.json`` (see
    ``hardening.read_stream_failures``) and prints anything new on the console,
    because this node's own stdout lives in a log file nobody reads mid-session.
    """

    def __init__(self, videos_dir: str, logger=None):
        self._path = os.path.join(videos_dir, hardening.STREAM_FAILURES_NAME)
        self._logger = logger
        self._lock = threading.Lock()
        self._failures: dict[str, dict] = {}

    def record(self, topic: str, reason: str, frames: int = 0, severity: str = "error"):
        with self._lock:
            self._failures[str(topic)] = {
                "topic": str(topic),
                "reason": str(reason),
                "frames": int(frames),
                "severity": str(severity),
                "at": datetime.now().isoformat(timespec="seconds"),
            }
            snapshot = dict(self._failures)
        try:
            hardening.write_atomic_json(
                self._path,
                {
                    "updated_at": datetime.now().isoformat(timespec="seconds"),
                    "failures": snapshot,
                },
            )
        except OSError as exc:  # pragma: no cover - disk full etc.
            if self._logger is not None:
                self._logger.warn(f"could not write {self._path}: {exc}")

    @property
    def failures(self) -> dict[str, dict]:
        with self._lock:
            return dict(self._failures)


class StreamRecorder:
    def __init__(
        self,
        topic: str,
        output_dir: str,
        crf: int,
        encoder: str,
        fps: float,
        max_queue: int,
        logger,
        resolver: EncoderResolver,
        failures: StreamFailureLog,
    ):
        self.topic = topic
        self.logger = logger
        self.frame_idx = 0
        self._proc = None
        self._csv_file = None
        self._csv_writer = None
        self._lock = threading.Lock()
        self._started = False
        self._frames = deque()
        self._cv = threading.Condition()
        self._stopping = False
        self._dropped = 0
        self._skipped = 0
        self._max_queue = max(1, max_queue)
        self._failures = failures
        self._failed = False
        self._retried = False

        stem = topic_to_filename(topic)
        self._video_path = os.path.join(output_dir, f"{stem}{hardening.VIDEO_EXT}")
        self._csv_path = os.path.join(output_dir, f"{stem}.csv")
        self._log_path = os.path.join(output_dir, f"{stem}.ffmpeg.log")
        self._crf = crf
        # Colour topics use h264_nvenc: covers both the standard
        # realsense2_camera convention (/color/) and the flat PoE DDS
        # convention (_Color).
        if ("/color/" in topic or "_Color" in topic) and encoder == "hevc_nvenc":
            requested = "h264_nvenc"
        else:
            requested = encoder
        # Probe once per encoder: an unusable NVENC falls back to libx264/libx265
        # instead of silently writing 0-byte files.
        decision = resolver.resolve(requested, topic)
        self._encoder = decision.effective
        self._requested_encoder = decision.requested
        self._retry_encoder = usable_fallback(resolver, requested, self._encoder)
        self._fps = fps
        self._frame_interval_ns = int(1_000_000_000 / fps) if fps > 0 else 0
        self._next_emit_ts_ns = 0
        self._stderr_file = None
        self._calibrating = True
        self._calib_frames: list[tuple[bytes, int, int, int]] = []
        self._width = 0
        self._height = 0
        self._pixel_fmt = ""
        self._mux_fps = fps
        self._worker = threading.Thread(target=self._writer_loop, daemon=True)
        self._worker.start()

    def _pixel_fmt_from_encoding(self, encoding: str) -> str:
        mapping = {
            "bgr8": "bgr24",
            "rgb8": "rgb24",
            "bgra8": "bgra",
            "rgba8": "rgba",
            "mono8": "gray",
            "mono16": "gray16le",
            "yuv422": "yuyv422",
            "yuv422_yuy2": "yuyv422",
            "yuyv": "yuyv422",
            "yuyv422": "yuyv422",
        }
        fmt = mapping.get(encoding.lower())
        if fmt is None:
            self.logger.warn(
                f"[{self.topic}] Unknown encoding '{encoding}', assuming bgr24."
            )
            fmt = "bgr24"
        return fmt

    def _row_bytes_from_encoding(self, encoding: str, width: int) -> int | None:
        bytes_per_pixel = {
            "bgr8": 3,
            "rgb8": 3,
            "bgra8": 4,
            "rgba8": 4,
            "mono8": 1,
            "mono16": 2,
            "yuv422": 2,
            "yuv422_yuy2": 2,
            "yuyv": 2,
            "yuyv422": 2,
        }
        bpp = bytes_per_pixel.get(encoding.lower())
        if bpp is None:
            return None
        return width * bpp

    def _normalize_buffer(self, msg: Image) -> bytes:
        row_bytes = self._row_bytes_from_encoding(msg.encoding, msg.width)
        if row_bytes is None or msg.step == row_bytes:
            return bytes(msg.data)

        data = msg.data
        out = bytearray(row_bytes * msg.height)
        for row in range(msg.height):
            start = row * msg.step
            end = start + row_bytes
            out[row * row_bytes : (row + 1) * row_bytes] = data[start:end]
        return bytes(out)

    def _open_outputs(self):
        self._csv_file = open(self._csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(["frame_idx", "ros_timestamp_ns"])

    def _spawn_ffmpeg(self):
        encoder_args = build_encoder_args(self._encoder, self._crf, self._mux_fps)

        cmd = [
            "ffmpeg",
            "-y",
            "-f",
            "rawvideo",
            "-vcodec",
            "rawvideo",
            "-pix_fmt",
            self._pixel_fmt,
            "-s",
            f"{self._width}x{self._height}",
            "-r",
            str(self._mux_fps),
            "-i",
            "pipe:0",
            *encoder_args,
            *build_muxer_args(hardening.VIDEO_EXT),
            self._video_path,
        ]

        self.logger.info(
            f"[{self.topic}] Starting ffmpeg -> {self._video_path}\n"
            f"  encoder={self._encoder} (requested {self._requested_encoder}) "
            f"crf={self._crf} "
            f"{self._width}x{self._height} @ {self._mux_fps}fps "
            f"(configured {self._fps}fps) "
            f"gop={keyframe_interval(self._mux_fps)} pixel_fmt={self._pixel_fmt}"
        )

        self._stderr_file = open(self._log_path, "w", encoding="utf-8")
        self._proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=self._stderr_file,
        )

    def _close_ffmpeg(self, wait_s: float = 30.0):
        """Close the current ffmpeg; safe to call on an already-dead process.

        ``wait_s`` matters: on a stop ffmpeg still has to flush its encoder
        pipeline and close the open Matroska cluster. That finalisation is much
        cheaper than the MP4 ``+faststart`` second pass it replaced, but killing
        ffmpeg early still costs the frames held in its buffers.
        """
        proc, self._proc = self._proc, None
        if proc is not None:
            try:
                if proc.stdin is not None:
                    proc.stdin.close()
            except Exception:
                pass
            try:
                proc.wait(timeout=wait_s)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass
        for handle in (self._csv_file, self._stderr_file):
            if handle is not None:
                try:
                    handle.close()
                except Exception:
                    pass
        self._csv_file = None
        self._stderr_file = None

    def _start(self, width: int, height: int, pixel_fmt: str, mux_fps: float):
        self._width = width
        self._height = height
        self._pixel_fmt = pixel_fmt
        self._mux_fps = mux_fps
        self._open_outputs()
        self._spawn_ffmpeg()
        self._started = True

    def _fail_stream(self, reason: str):
        self._failed = True
        self.logger.error(f"[{self.topic}] VIDEO LOSS: {reason}")
        self._failures.record(self.topic, reason, self.frame_idx, severity="error")

    def _retry_with_software_encoder(self, frame) -> bool:
        """One software-encoder restart, only if nothing was written yet.

        Restarting a stream that has already produced frames would discard the
        existing output (``ffmpeg -y``), so this refuses to do that:
        mid-session death is reported as a loss instead.
        """
        with self._lock:
            if (
                self._stopping
                or self._retry_encoder is None
                or self._retried
                or self.frame_idx > 0
            ):
                return False
            self._retried = True
            failed_encoder = self._encoder
            self._encoder = self._retry_encoder
        attempt_log = f"{self._log_path}.attempt1"
        try:
            self._close_ffmpeg()
            if os.path.exists(self._log_path):
                os.replace(self._log_path, attempt_log)
            self._open_outputs()
            self._spawn_ffmpeg()
        except Exception as exc:
            self._fail_stream(
                f"software-encoder retry ({self._encoder}) could not start: {exc}"
            )
            return False
        reason = (
            f"encoder {failed_encoder} died before writing any frame; "
            f"retried once with {self._encoder}"
        )
        self.logger.error(f"[{self.topic}] {reason}. First failure log: {attempt_log}")
        self._failures.record(self.topic, reason, 0, severity="warn")
        with self._cv:
            self._frames.appendleft(frame)
        return True

    def write(self, msg: Image):
        ts_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        with self._lock:
            if self._frame_interval_ns > 0:
                if self._next_emit_ts_ns == 0:
                    self._next_emit_ts_ns = ts_ns
                if ts_ns < self._next_emit_ts_ns:
                    self._skipped += 1
                    return
                self._next_emit_ts_ns += self._frame_interval_ns

            buffer = self._normalize_buffer(msg)

            if self._calibrating:
                self._calib_frames.append((buffer, ts_ns, msg.width, msg.height))
                if not calibration_done([f[1] for f in self._calib_frames]):
                    return
                self._calibrating = False
                mux_fps = sustained_fps(
                    [f[1] for f in self._calib_frames], fallback_fps=self._fps
                )
                if mux_fps <= 0:
                    mux_fps = self._fps
                else:
                    mux_fps = min(self._fps, round(mux_fps, 2))
                    if mux_fps < self._fps:
                        self.logger.warn(
                            f"[{self.topic}] Configured {self._fps}fps but only "
                            f"{mux_fps}fps sustained after the first "
                            f"{CALIBRATION_SETTLE_S:.0f}s; muxing at that rate to "
                            "keep playback speed correct."
                        )
                pixel_fmt = self._pixel_fmt_from_encoding(msg.encoding)
                self._start(msg.width, msg.height, pixel_fmt, mux_fps)
                pending = self._calib_frames
                self._calib_frames = []
                cap_pending = False
            elif not self._started:
                return  # unreachable once calibration has run, but guards ordering
            else:
                pending = [(buffer, ts_ns, msg.width, msg.height)]
                cap_pending = True

        with self._cv:
            for frame in pending:
                if cap_pending and len(self._frames) >= self._max_queue:
                    self._frames.popleft()
                    self._dropped += 1
                self._frames.append(frame)
            self._cv.notify()

    def _writer_loop(self):
        while True:
            with self._cv:
                while not self._frames and not self._stopping:
                    self._cv.wait()
                if self._stopping and not self._frames:
                    return
                frame = self._frames.popleft()

            if self._proc is None or self._proc.poll() is not None:
                if self._retry_with_software_encoder(frame):
                    continue
                self._fail_stream(
                    f"ffmpeg exited early (encoder={self._encoder}) after "
                    f"{self.frame_idx} frames. See {self._log_path}"
                )
                return

            try:
                self._proc.stdin.write(frame[0])
            except BrokenPipeError:
                if self._retry_with_software_encoder(frame):
                    continue
                self._fail_stream(
                    f"ffmpeg pipe closed unexpectedly (encoder={self._encoder}) after "
                    f"{self.frame_idx} frames. See {self._log_path}"
                )
                return

            self._csv_writer.writerow([self.frame_idx, frame[1]])
            self.frame_idx += 1

    def stop(self):
        if self._started:
            with self._lock:
                if rclpy.ok():
                    self.logger.info(
                        f"[{self.topic}] Stopping - {self.frame_idx} frames written."
                    )
            with self._cv:
                self._stopping = True
                self._cv.notify_all()
            self._worker.join(timeout=5.0)
            self._close_ffmpeg()
            if self._dropped and rclpy.ok():
                self.logger.warn(
                    f"[{self.topic}] Dropped {self._dropped} frames due to backpressure."
                )

        size = file_size(self._video_path)
        if self._failed or size == 0:
            self._fail_stream(
                f"video file is {size} bytes after {self.frame_idx} frames "
                f"({self._video_path})"
            )
        elif rclpy.ok():
            self.logger.info(
                f"[{self.topic}] Saved ({size} bytes):\n"
                f"  video: {self._video_path}\n"
                f"  timestamps: {self._csv_path}"
            )


class CompressedStreamRecorder:
    """Pipe pre-compressed JPEG frames straight into ffmpeg for transcoding.

    The camera publishes CompressedImage (JPEG). Subscribing to the compressed
    topic uses ~10x less network bandwidth than the raw Color topic, which is
    the difference between fitting and not fitting a multi-stream session on
    a gigabit camera link.
    """

    def __init__(
        self,
        topic: str,
        output_dir: str,
        crf: int,
        encoder: str,
        fps: float,
        max_queue: int,
        logger,
        resolver: EncoderResolver,
        failures: StreamFailureLog,
    ):
        self.topic = topic
        self.logger = logger
        self.frame_idx = 0
        self._proc = None
        self._csv_file = None
        self._csv_writer = None
        self._lock = threading.Lock()
        self._started = False
        self._frames = deque()
        self._cv = threading.Condition()
        self._stopping = False
        self._dropped = 0
        self._skipped = 0
        self._max_queue = max(1, max_queue)
        self._failures = failures
        self._failed = False
        self._retried = False

        stem = topic_to_filename(topic)
        self._video_path = os.path.join(output_dir, f"{stem}{hardening.VIDEO_EXT}")
        self._csv_path = os.path.join(output_dir, f"{stem}.csv")
        self._log_path = os.path.join(output_dir, f"{stem}.ffmpeg.log")
        self._crf = crf
        requested = "h264_nvenc" if encoder == "hevc_nvenc" else encoder
        decision = resolver.resolve(requested, topic)
        self._encoder = decision.effective
        self._requested_encoder = decision.requested
        self._retry_encoder = usable_fallback(resolver, requested, self._encoder)
        self._fps = fps
        self._frame_interval_ns = int(1_000_000_000 / fps) if fps > 0 else 0
        self._next_emit_ts_ns = 0
        self._stderr_file = None
        self._calibrating = True
        self._calib_frames: list[tuple[bytes, int]] = []
        self._mux_fps = fps
        self._input_format = ""
        self._worker = threading.Thread(target=self._writer_loop, daemon=True)
        self._worker.start()

    def _open_outputs(self):
        self._csv_file = open(self._csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(["frame_idx", "ros_timestamp_ns"])

    def _spawn_ffmpeg(self):
        encoder_args = build_encoder_args(self._encoder, self._crf, self._mux_fps)

        cmd = [
            "ffmpeg",
            "-y",
            "-f",
            "image2pipe",
            "-framerate",
            str(self._mux_fps),
            "-vcodec",
            "mjpeg",
            "-i",
            "pipe:0",
            *encoder_args,
            *build_muxer_args(hardening.VIDEO_EXT),
            self._video_path,
        ]

        self.logger.info(
            f"[{self.topic}] Starting ffmpeg (compressed) -> {self._video_path}\n"
            f"  encoder={self._encoder} (requested {self._requested_encoder}) "
            f"crf={self._crf} @ {self._mux_fps}fps "
            f"(configured {self._fps}fps) gop={keyframe_interval(self._mux_fps)} "
            f"input_format={self._input_format}"
        )

        self._stderr_file = open(self._log_path, "w", encoding="utf-8")
        self._proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=self._stderr_file,
        )

    def _close_ffmpeg(self, wait_s: float = 30.0):
        """Close the current ffmpeg; ``wait_s`` covers the final cluster flush."""
        proc, self._proc = self._proc, None
        if proc is not None:
            try:
                if proc.stdin is not None:
                    proc.stdin.close()
            except Exception:
                pass
            try:
                proc.wait(timeout=wait_s)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass
        for handle in (self._csv_file, self._stderr_file):
            if handle is not None:
                try:
                    handle.close()
                except Exception:
                    pass
        self._csv_file = None
        self._stderr_file = None

    def _start(self, fmt_hint: str, mux_fps: float):
        self._input_format = fmt_hint
        self._mux_fps = mux_fps
        self._open_outputs()
        self._spawn_ffmpeg()
        self._started = True

    def _fail_stream(self, reason: str):
        self._failed = True
        self.logger.error(f"[{self.topic}] VIDEO LOSS: {reason}")
        self._failures.record(self.topic, reason, self.frame_idx, severity="error")

    def _retry_with_software_encoder(self, frame) -> bool:
        with self._lock:
            if (
                self._stopping
                or self._retry_encoder is None
                or self._retried
                or self.frame_idx > 0
            ):
                return False
            self._retried = True
            failed_encoder = self._encoder
            self._encoder = self._retry_encoder
        attempt_log = f"{self._log_path}.attempt1"
        try:
            self._close_ffmpeg()
            if os.path.exists(self._log_path):
                os.replace(self._log_path, attempt_log)
            self._open_outputs()
            self._spawn_ffmpeg()
        except Exception as exc:
            self._fail_stream(
                f"software-encoder retry ({self._encoder}) could not start: {exc}"
            )
            return False
        reason = (
            f"encoder {failed_encoder} died before writing any frame; "
            f"retried once with {self._encoder}"
        )
        self.logger.error(f"[{self.topic}] {reason}. First failure log: {attempt_log}")
        self._failures.record(self.topic, reason, 0, severity="warn")
        with self._cv:
            self._frames.appendleft(frame)
        return True

    def write(self, msg: CompressedImage):
        ts_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        with self._lock:
            if self._frame_interval_ns > 0:
                if self._next_emit_ts_ns == 0:
                    self._next_emit_ts_ns = ts_ns
                if ts_ns < self._next_emit_ts_ns:
                    self._skipped += 1
                    return
                self._next_emit_ts_ns += self._frame_interval_ns

            buffer = bytes(msg.data)

            if self._calibrating:
                self._calib_frames.append((buffer, ts_ns))
                if not calibration_done([f[1] for f in self._calib_frames]):
                    return
                self._calibrating = False
                mux_fps = sustained_fps(
                    [f[1] for f in self._calib_frames], fallback_fps=self._fps
                )
                if mux_fps <= 0:
                    mux_fps = self._fps
                else:
                    mux_fps = min(self._fps, round(mux_fps, 2))
                    if mux_fps < self._fps:
                        self.logger.warn(
                            f"[{self.topic}] Configured {self._fps}fps but only "
                            f"{mux_fps}fps sustained after the first "
                            f"{CALIBRATION_SETTLE_S:.0f}s; muxing at that rate to "
                            "keep playback speed correct."
                        )
                fmt = msg.format.lower() if msg.format else ""
                if "jpeg" not in fmt and "jpg" not in fmt:
                    self.logger.warn(
                        f"[{self.topic}] format='{msg.format}' is not JPEG; "
                        "ffmpeg may fail to decode."
                    )
                self._start(msg.format or "unknown", mux_fps)
                pending = self._calib_frames
                self._calib_frames = []
                cap_pending = False
            elif not self._started:
                return  # unreachable once calibration has run, but guards ordering
            else:
                pending = [(buffer, ts_ns)]
                cap_pending = True

        with self._cv:
            for frame in pending:
                if cap_pending and len(self._frames) >= self._max_queue:
                    self._frames.popleft()
                    self._dropped += 1
                self._frames.append(frame)
            self._cv.notify()

    def _writer_loop(self):
        while True:
            with self._cv:
                while not self._frames and not self._stopping:
                    self._cv.wait()
                if self._stopping and not self._frames:
                    return
                frame = self._frames.popleft()

            if self._proc is None or self._proc.poll() is not None:
                if self._retry_with_software_encoder(frame):
                    continue
                self._fail_stream(
                    f"ffmpeg exited early (encoder={self._encoder}) after "
                    f"{self.frame_idx} frames. See {self._log_path}"
                )
                return

            try:
                self._proc.stdin.write(frame[0])
            except BrokenPipeError:
                if self._retry_with_software_encoder(frame):
                    continue
                self._fail_stream(
                    f"ffmpeg pipe closed unexpectedly (encoder={self._encoder}) after "
                    f"{self.frame_idx} frames. See {self._log_path}"
                )
                return

            self._csv_writer.writerow([self.frame_idx, frame[1]])
            self.frame_idx += 1

    def stop(self):
        if self._started:
            with self._lock:
                if rclpy.ok():
                    self.logger.info(
                        f"[{self.topic}] Stopping - {self.frame_idx} frames written."
                    )
            with self._cv:
                self._stopping = True
                self._cv.notify_all()
            self._worker.join(timeout=5.0)
            self._close_ffmpeg()
            if self._dropped and rclpy.ok():
                self.logger.warn(
                    f"[{self.topic}] Dropped {self._dropped} frames due to backpressure."
                )

        size = file_size(self._video_path)
        if self._failed or size == 0:
            self._fail_stream(
                f"video file is {size} bytes after {self.frame_idx} frames "
                f"({self._video_path})"
            )
        elif rclpy.ok():
            self.logger.info(
                f"[{self.topic}] Saved ({size} bytes):\n"
                f"  video: {self._video_path}\n"
                f"  timestamps: {self._csv_path}"
            )


class ColourVideoRecorder(Node):
    def __init__(self):
        super().__init__("colour_video_recorder")

        self.declare_parameter("topics", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("compressed_topics", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("topic_fps_overrides", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("output_dir", os.path.expanduser("~/recordings"))
        self.declare_parameter("crf", 18)
        self.declare_parameter("encoder", "hevc_nvenc")
        self.declare_parameter("software_fallback", True)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("queue_size", 5)
        self.declare_parameter("max_queue", 30)

        topics = [str(t) for t in self.get_parameter("topics").value if t]
        compressed_topics = [
            str(t) for t in self.get_parameter("compressed_topics").value if t
        ]
        override_entries = [
            str(t) for t in self.get_parameter("topic_fps_overrides").value if t
        ]
        output_dir = str(self.get_parameter("output_dir").value)
        crf = int(self.get_parameter("crf").value)
        encoder = str(self.get_parameter("encoder").value)
        software_fallback = bool(self.get_parameter("software_fallback").value)
        fps = float(self.get_parameter("fps").value)
        queue_size = int(self.get_parameter("queue_size").value)
        max_queue = int(self.get_parameter("max_queue").value)

        fps_overrides: dict = {}
        for entry in override_entries:
            topic, _, value = entry.rpartition(":")
            if topic and value:
                try:
                    fps_overrides[topic] = float(value)
                except ValueError:
                    self.get_logger().warn(
                        f"Ignoring malformed topic_fps_overrides entry '{entry}'"
                    )

        if not topics and not compressed_topics:
            raise RuntimeError("No topics provided for recording")

        os.makedirs(output_dir, exist_ok=True)

        # Probe the encoders once, before any ffmpeg is spawned; the whole point
        # of the 2026-10-01 fix is that this cannot silently fail later.
        self._resolver = EncoderResolver(
            requested=encoder, allow_software_fallback=software_fallback
        )
        self._failures = StreamFailureLog(output_dir, self.get_logger())
        self._output_dir = output_dir

        self._recorders = {}
        self._subscriptions = []
        self._qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=queue_size,
        )

        for topic in topics:
            topic_fps = fps_overrides.get(topic, fps)
            recorder = StreamRecorder(
                topic,
                output_dir,
                crf,
                encoder,
                topic_fps,
                max_queue,
                self.get_logger(),
                self._resolver,
                self._failures,
            )
            self._recorders[topic] = recorder
            self._subscriptions.append(
                self.create_subscription(
                    Image,
                    topic,
                    recorder.write,
                    self._qos,
                )
            )

        for topic in compressed_topics:
            topic_fps = fps_overrides.get(topic, fps)
            recorder = CompressedStreamRecorder(
                topic,
                output_dir,
                crf,
                encoder,
                topic_fps,
                max_queue,
                self.get_logger(),
                self._resolver,
                self._failures,
            )
            self._recorders[topic] = recorder
            self._subscriptions.append(
                self.create_subscription(
                    CompressedImage,
                    topic,
                    recorder.write,
                    self._qos,
                )
            )

        self._write_encoder_status()
        self._log_encoder_summary()

        self.get_logger().info(
            f"Recording {len(topics)} raw + {len(compressed_topics)} compressed topics to {output_dir}"
            + (f"\n  fps overrides: {fps_overrides}" if fps_overrides else "")
        )

    def _write_encoder_status(self):
        """Hand the manager the encoder verdicts (console alarm + forensics)."""
        payload = self._resolver.status_payload(
            datetime.now().isoformat(timespec="seconds")
        )
        path = os.path.join(self._output_dir, hardening.ENCODER_STATUS_NAME)
        try:
            hardening.write_atomic_json(path, payload)
        except OSError as exc:  # pragma: no cover - disk full etc.
            self.get_logger().warn(f"could not write encoder status to {path}: {exc}")

    def _log_encoder_summary(self):
        for stream, decision in self._resolver.decisions.items():
            if decision.ok and not decision.fallback_used:
                self.get_logger().info(f"[{stream}] {decision.summary()}")
            else:
                self.get_logger().error(f"[{stream}] {decision.summary()}")
        if self._resolver.fallback_used:
            self.get_logger().error(
                "SOFTWARE ENCODER FALLBACK ACTIVE: NVENC could not initialise; "
                f"using {self._resolver.effective_encoders}. "
                "Video will still be recorded, on the CPU."
            )

    def destroy_node(self):
        for recorder in self._recorders.values():
            recorder.stop()
        for subscription in list(self._subscriptions):
            try:
                self.destroy_subscription(subscription)
            except ValueError:
                pass
        failures = self._failures.failures
        if failures:
            self.get_logger().error(
                f"{len(failures)} video stream(s) failed: "
                + ", ".join(sorted(failures))
                + f" (see {hardening.STREAM_FAILURES_NAME})"
            )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    # SIGTERM/SIGHUP should finalise the Matroska file rather than truncate it;
    # the manager forwards SIGINT on a graceful stop, but a supervisor or a
    # killed terminal can send TERM/HUP directly. Installed before node
    # construction so the encoder probe is covered too.
    stop = hardening.StopRequest()
    hardening.install_stop_handlers(stop)
    node = ColourVideoRecorder()
    try:
        while rclpy.ok() and not stop.requested:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    except hardening.ros_shutdown_exceptions():
        pass
    finally:
        if stop.requested:
            node.get_logger().warn(f"Stopping video recorder: {stop.reason()}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
