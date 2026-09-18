#!/usr/bin/env python3
"""H.265 video recorder for camera colour/IR streams (any sensor)."""

import csv
import os
import subprocess
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from session_recorder.video_encoding import build_encoder_args, keyframe_interval


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


def measured_fps(timestamps_ns: list[int]) -> float:
    if len(timestamps_ns) < 2:
        return 0.0
    span_s = (timestamps_ns[-1] - timestamps_ns[0]) / 1e9
    return (len(timestamps_ns) - 1) / span_s if span_s > 0 else 0.0


def calibration_done(timestamps_ns: list[int]) -> bool:
    if len(timestamps_ns) < 2:
        return False
    elapsed = (timestamps_ns[-1] - timestamps_ns[0]) / 1e9
    if len(timestamps_ns) >= CALIBRATION_MIN_FRAMES and elapsed >= CALIBRATION_WINDOW_S:
        return True
    return elapsed >= CALIBRATION_MAX_WAIT_S


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

        stem = topic_to_filename(topic)
        self._video_path = os.path.join(output_dir, f"{stem}.mp4")
        self._csv_path = os.path.join(output_dir, f"{stem}.csv")
        self._log_path = os.path.join(output_dir, f"{stem}.ffmpeg.log")
        self._crf = crf
        # Colour topics use h264_nvenc: covers both the standard
        # realsense2_camera convention (/color/) and the flat PoE DDS
        # convention (_Color).
        if ("/color/" in topic or "_Color" in topic) and encoder == "hevc_nvenc":
            self._encoder = "h264_nvenc"
        else:
            self._encoder = encoder
        self._fps = fps
        self._frame_interval_ns = int(1_000_000_000 / fps) if fps > 0 else 0
        self._next_emit_ts_ns = 0
        self._stderr_file = None
        self._calibrating = True
        self._calib_frames: list[tuple[bytes, int, int, int]] = []
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

    def _start(self, width: int, height: int, pixel_fmt: str, mux_fps: float):
        self._csv_file = open(self._csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(["frame_idx", "ros_timestamp_ns"])

        encoder_args = build_encoder_args(self._encoder, self._crf, mux_fps)

        cmd = [
            "ffmpeg",
            "-y",
            "-f",
            "rawvideo",
            "-vcodec",
            "rawvideo",
            "-pix_fmt",
            pixel_fmt,
            "-s",
            f"{width}x{height}",
            "-r",
            str(mux_fps),
            "-i",
            "pipe:0",
            *encoder_args,
            "-movflags",
            "+faststart",
            self._video_path,
        ]

        self.logger.info(
            f"[{self.topic}] Starting ffmpeg -> {self._video_path}\n"
            f"  encoder={self._encoder} crf={self._crf} "
            f"{width}x{height} @ {mux_fps}fps (configured {self._fps}fps) "
            f"gop={keyframe_interval(mux_fps)} pixel_fmt={pixel_fmt}"
        )

        self._stderr_file = open(self._log_path, "w", encoding="utf-8")
        self._proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=self._stderr_file,
        )
        self._started = True

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
                mux_fps = measured_fps([f[1] for f in self._calib_frames])
                if mux_fps <= 0:
                    mux_fps = self._fps
                else:
                    mux_fps = min(self._fps, round(mux_fps, 2))
                    if mux_fps < self._fps:
                        self.logger.warn(
                            f"[{self.topic}] Configured {self._fps}fps but only "
                            f"measured {mux_fps}fps arriving; muxing at the "
                            "measured rate to keep playback speed correct."
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
                buffer, ts_ns, width, height = self._frames.popleft()

            if self._proc is None or self._proc.poll() is not None:
                self.logger.error(
                    f"[{self.topic}] ffmpeg exited early. See {self._log_path}"
                )
                return

            try:
                self._proc.stdin.write(buffer)
            except BrokenPipeError:
                self.logger.error(
                    f"[{self.topic}] ffmpeg pipe closed unexpectedly. See {self._log_path}"
                )
                return

            self._csv_writer.writerow([self.frame_idx, ts_ns])
            self.frame_idx += 1

    def stop(self):
        with self._lock:
            if not self._started:
                return
            if rclpy.ok():
                self.logger.info(
                    f"[{self.topic}] Stopping - {self.frame_idx} frames written."
                )
        with self._cv:
            self._stopping = True
            self._cv.notify_all()
        self._worker.join(timeout=5.0)
        try:
            self._proc.stdin.close()
            self._proc.wait(timeout=30)
        except Exception as exc:
            if rclpy.ok():
                self.logger.warn(f"[{self.topic}] ffmpeg shutdown error: {exc}")
            self._proc.kill()
        if self._csv_file:
            self._csv_file.close()
        if self._stderr_file:
            self._stderr_file.close()
        if self._dropped and rclpy.ok():
            self.logger.warn(
                f"[{self.topic}] Dropped {self._dropped} frames due to backpressure."
            )
        if rclpy.ok():
            self.logger.info(
                f"[{self.topic}] Saved:\n"
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

        stem = topic_to_filename(topic)
        self._video_path = os.path.join(output_dir, f"{stem}.mp4")
        self._csv_path = os.path.join(output_dir, f"{stem}.csv")
        self._log_path = os.path.join(output_dir, f"{stem}.ffmpeg.log")
        self._crf = crf
        self._encoder = "h264_nvenc" if encoder == "hevc_nvenc" else encoder
        self._fps = fps
        self._frame_interval_ns = int(1_000_000_000 / fps) if fps > 0 else 0
        self._next_emit_ts_ns = 0
        self._stderr_file = None
        self._calibrating = True
        self._calib_frames: list[tuple[bytes, int]] = []
        self._worker = threading.Thread(target=self._writer_loop, daemon=True)
        self._worker.start()

    def _start(self, fmt_hint: str, mux_fps: float):
        self._csv_file = open(self._csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(["frame_idx", "ros_timestamp_ns"])

        encoder_args = build_encoder_args(self._encoder, self._crf, mux_fps)

        cmd = [
            "ffmpeg",
            "-y",
            "-f",
            "image2pipe",
            "-framerate",
            str(mux_fps),
            "-vcodec",
            "mjpeg",
            "-i",
            "pipe:0",
            *encoder_args,
            "-movflags",
            "+faststart",
            self._video_path,
        ]

        self.logger.info(
            f"[{self.topic}] Starting ffmpeg (compressed) -> {self._video_path}\n"
            f"  encoder={self._encoder} crf={self._crf} @ {mux_fps}fps "
            f"(configured {self._fps}fps) gop={keyframe_interval(mux_fps)} "
            f"input_format={fmt_hint}"
        )

        self._stderr_file = open(self._log_path, "w", encoding="utf-8")
        self._proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=self._stderr_file,
        )
        self._started = True

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
                mux_fps = measured_fps([f[1] for f in self._calib_frames])
                if mux_fps <= 0:
                    mux_fps = self._fps
                else:
                    mux_fps = min(self._fps, round(mux_fps, 2))
                    if mux_fps < self._fps:
                        self.logger.warn(
                            f"[{self.topic}] Configured {self._fps}fps but only "
                            f"measured {mux_fps}fps arriving; muxing at the "
                            "measured rate to keep playback speed correct."
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
                buffer, ts_ns = self._frames.popleft()

            if self._proc is None or self._proc.poll() is not None:
                self.logger.error(
                    f"[{self.topic}] ffmpeg exited early. See {self._log_path}"
                )
                return

            try:
                self._proc.stdin.write(buffer)
            except BrokenPipeError:
                self.logger.error(
                    f"[{self.topic}] ffmpeg pipe closed unexpectedly. See {self._log_path}"
                )
                return

            self._csv_writer.writerow([self.frame_idx, ts_ns])
            self.frame_idx += 1

    def stop(self):
        with self._lock:
            if not self._started:
                return
            if rclpy.ok():
                self.logger.info(
                    f"[{self.topic}] Stopping - {self.frame_idx} frames written."
                )
        with self._cv:
            self._stopping = True
            self._cv.notify_all()
        self._worker.join(timeout=5.0)
        try:
            self._proc.stdin.close()
            self._proc.wait(timeout=30)
        except Exception as exc:
            if rclpy.ok():
                self.logger.warn(f"[{self.topic}] ffmpeg shutdown error: {exc}")
            self._proc.kill()
        if self._csv_file:
            self._csv_file.close()
        if self._stderr_file:
            self._stderr_file.close()
        if self._dropped and rclpy.ok():
            self.logger.warn(
                f"[{self.topic}] Dropped {self._dropped} frames due to backpressure."
            )
        if rclpy.ok():
            self.logger.info(
                f"[{self.topic}] Saved:\n"
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
                topic, output_dir, crf, encoder, topic_fps, max_queue, self.get_logger()
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
                topic, output_dir, crf, encoder, topic_fps, max_queue, self.get_logger()
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

        self.get_logger().info(
            f"Recording {len(topics)} raw + {len(compressed_topics)} compressed topics to {output_dir}"
            + (f"\n  fps overrides: {fps_overrides}" if fps_overrides else "")
        )

    def destroy_node(self):
        for recorder in self._recorders.values():
            recorder.stop()
        for subscription in list(self._subscriptions):
            try:
                self.destroy_subscription(subscription)
            except ValueError:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ColourVideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
