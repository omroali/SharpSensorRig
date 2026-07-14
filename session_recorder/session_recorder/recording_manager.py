#!/usr/bin/env python3
"""Unified service-driven recording manager for the whole sensor rig.

One /start_recording -> /stop_recording pair records every sensor marked
record: true in the unified config directory: Kinect + RealSense colour to
H.265 videos, and depth/IR/camera_info/velodyne/vicon/TF into one rosbag.
Each take lands in a single session directory:

  <output_root>/<uuid>/session_<stamp>/
    videos/*.mp4 + per-stream timestamp CSVs
    bag/         one mcap with every non-video topic
"""

import os
import random
import shutil
import signal
import socket
import subprocess
import threading
import time
from datetime import datetime

import rclpy
import yaml
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from session_recorder.topics import parse_expected_entry


class RecordingManager(Node):
    def __init__(self):
        super().__init__("recording_manager")

        self.declare_parameter("output_root", os.path.expanduser("~/data"))
        self.declare_parameter("participant_id", "0000")
        self.declare_parameter("color_topics", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("color_remaps", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("color_compressed_topics", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("topic_fps_overrides", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("bag_topics", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("bag_regex", "")
        self.declare_parameter("bag_exclude_regex", "")
        self.declare_parameter("bag_throttles", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("expected_topics", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("expected_regex", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("calib_source_dirs", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("session_check_enabled", True)
        self.declare_parameter("session_check_min_rate_factor", 0.5)
        self.declare_parameter("video_encoder", "hevc_nvenc")
        self.declare_parameter("video_crf", 18)
        self.declare_parameter("video_fps", 30.0)
        self.declare_parameter("bag_storage", "mcap")
        self.declare_parameter("bag_compression", "none")
        self.declare_parameter("mcap_preset", "zstd_fast")
        self.declare_parameter("bag_cache_size_mb", 128)
        self.declare_parameter("nexus_capture", False)
        self.declare_parameter("nexus_host", "192.168.10.1")
        self.declare_parameter("nexus_port", 3030)
        self.declare_parameter("nexus_path", "")

        self._output_root = str(self.get_parameter("output_root").value)
        self._participant_id = str(self.get_parameter("participant_id").value)
        self._color_topics = [str(t) for t in self.get_parameter("color_topics").value if t]
        self._color_remaps = [str(t) for t in self.get_parameter("color_remaps").value if t]
        self._color_compressed_topics = [str(t) for t in self.get_parameter("color_compressed_topics").value if t]
        self._topic_fps_overrides = [str(t) for t in self.get_parameter("topic_fps_overrides").value if t]
        self._bag_topics = list(
            dict.fromkeys(str(t) for t in self.get_parameter("bag_topics").value if t)
        )
        self._bag_regex = str(self.get_parameter("bag_regex").value)
        self._bag_exclude_regex = str(self.get_parameter("bag_exclude_regex").value)
        self._bag_throttles = [str(t) for t in self.get_parameter("bag_throttles").value if t]
        self._expected_topics = [str(t) for t in self.get_parameter("expected_topics").value if t]
        self._expected_regex = [str(t) for t in self.get_parameter("expected_regex").value if t]
        self._calib_source_dirs = [str(t) for t in self.get_parameter("calib_source_dirs").value if t]
        self._session_check_enabled = bool(self.get_parameter("session_check_enabled").value)
        self._session_check_min_rate_factor = float(
            self.get_parameter("session_check_min_rate_factor").value
        )
        self._video_encoder = str(self.get_parameter("video_encoder").value)
        self._video_crf = int(self.get_parameter("video_crf").value)
        self._video_fps = float(self.get_parameter("video_fps").value)
        self._bag_storage = str(self.get_parameter("bag_storage").value)
        self._bag_cache_size_mb = int(self.get_parameter("bag_cache_size_mb").value)
        self._bag_compression = str(self.get_parameter("bag_compression").value).lower()
        if self._bag_compression not in {"none", "storage", "file", "message"}:
            self.get_logger().warn(
                f"bag_compression='{self._bag_compression}' not recognized; "
                "treating as 'none'. Valid values: none, storage, file, message."
            )
            self._bag_compression = "none"

        self._mcap_preset = str(self.get_parameter("mcap_preset").value).lower()
        if self._mcap_preset not in {"fastwrite", "zstd_fast", "zstd_small"}:
            self.get_logger().warn(
                f"mcap_preset='{self._mcap_preset}' not recognized; "
                "falling back to 'zstd_fast'. Valid values: fastwrite, zstd_fast, zstd_small."
            )
            self._mcap_preset = "zstd_fast"

        self._lock = threading.Lock()
        self._active = False
        self._session_dir = ""
        self._session_start = 0.0
        self._color_proc = None
        self._bag_proc = None
        self._throttle_proc = None
        self._color_log = None
        self._bag_log = None
        self._throttle_log = None
        self._nexus_capture = bool(self.get_parameter("nexus_capture").value)
        self._nexus_host = str(self.get_parameter("nexus_host").value)
        self._nexus_port = int(self.get_parameter("nexus_port").value)
        self._nexus_path = str(self.get_parameter("nexus_path").value)
        self._nexus_trial_name = ""

        self._start_service = self.create_service(Trigger, "start_recording", self._on_start)
        self._stop_service = self.create_service(Trigger, "stop_recording", self._on_stop)

        self.get_logger().info(
            "Unified recording manager ready\n"
            f"  color_topics      : {self._color_topics}\n"
            f"  compressed_topics : {self._color_compressed_topics}\n"
            f"  fps_overrides     : {self._topic_fps_overrides}\n"
            f"  bag_topics        : {self._bag_topics}\n"
            f"  bag_regex         : {self._bag_regex or '<none>'}\n"
            f"  bag_exclude_regex : {self._bag_exclude_regex or '<none>'}\n"
            f"  bag_throttles     : {self._bag_throttles or '<none>'}\n"
            f"  calib_snapshots   : {self._calib_source_dirs or '<none>'}\n"
            f"  session_check     : {self._session_check_enabled}\n"
            f"  output_root       : {self._output_root}\n"
            f"  participant       : {self._participant_id}\n"
            f"  encoder           : {self._video_encoder}\n"
            f"  crf               : {self._video_crf}\n"
            f"  bag_storage       : {self._bag_storage}\n"
            f"  bag_compression   : {self._bag_compression}\n"
            f"  mcap_preset       : {self._mcap_preset}\n"
            f"  cache_size_mb     : {self._bag_cache_size_mb}\n"
            f"  nexus_capture     : {self._nexus_capture}\n"
            f"  nexus_host        : {self._nexus_host}\n"
            f"  nexus_port        : {self._nexus_port}\n"
            f"  nexus_path        : {self._nexus_path}"
        )

    def _session_path(self) -> str:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(self._output_root, self._participant_id, f"session_{stamp}")

    def _unique_bag_dir(self, base_dir: str) -> str:
        if not os.path.exists(base_dir):
            return base_dir
        index = 1
        while True:
            candidate = f"{base_dir}_{index}"
            if not os.path.exists(candidate):
                return candidate
            index += 1

    def _start_process(self, cmd: list[str], log_path: str):
        log_file = open(log_path, "w", encoding="utf-8")
        proc = subprocess.Popen(
            cmd,
            stdout=log_file,
            stderr=log_file,
            preexec_fn=os.setsid,
        )
        return proc, log_file

    def _build_color_cmd(self, output_dir: str) -> list[str]:
        # ros2 run -p name:=[] leaves the param uninitialized (empty STRING_ARRAY
        # parsing quirk). Use a single empty-string element as a sentinel; the
        # recorder filters falsy entries on read.
        def yaml_array(values):
            return "[" + ",".join(values or ['""']) + "]"

        cmd = [
            "ros2", "run", "session_recorder", "colour_video_recorder",
            "--ros-args",
            "-p", f"topics:={yaml_array(self._color_topics)}",
            "-p", f"compressed_topics:={yaml_array(self._color_compressed_topics)}",
            "-p", f"topic_fps_overrides:={yaml_array(self._topic_fps_overrides)}",
            "-p", f"output_dir:={output_dir}",
            "-p", f"encoder:={self._video_encoder}",
            "-p", f"crf:={self._video_crf}",
            "-p", f"fps:={self._video_fps}",
        ]

        for remap in self._color_remaps:
            cmd.extend(["-r", remap])

        return cmd

    def _build_bag_cmd(self, bag_dir: str) -> list[str]:
        cmd = [
            "ros2", "bag", "record",
            "-o", bag_dir,
            "--storage", self._bag_storage,
            "--max-cache-size", str(self._bag_cache_size_mb * 1024 * 1024),
        ]
        if self._bag_compression == "storage":
            if self._bag_storage == "mcap":
                cmd += ["--storage-preset-profile", self._mcap_preset]
            else:
                self.get_logger().warn(
                    f"bag_compression='storage' requires bag_storage='mcap'; "
                    f"current bag_storage='{self._bag_storage}'. Recording uncompressed."
                )
        elif self._bag_compression in {"file", "message"}:
            cmd += [
                "--compression-mode", self._bag_compression,
                "--compression-format", "zstd",
            ]
        if self._bag_regex:
            cmd += ["--regex", self._bag_regex]
        if self._bag_exclude_regex:
            cmd += ["--exclude-regex", self._bag_exclude_regex]
        if self._bag_topics:
            cmd += ["--topics", *self._bag_topics]
        return cmd

    def _build_throttle_cmd(self) -> list[str]:
        throttles = "[" + ",".join(self._bag_throttles) + "]"
        return [
            "ros2", "run", "session_recorder", "topic_throttle",
            "--ros-args", "-p", f"throttles:={throttles}",
        ]

    def _write_manifest(self, session_dir: str):
        """Persist what this session is supposed to contain; session_check
        and post-hoc tooling validate the recorded data against this."""
        expected = []
        for entry in self._expected_topics:
            topic, hz, sink = parse_expected_entry(entry)
            expected.append({"topic": topic, "expected_hz": hz, "sink": sink})
        expected_regex = []
        for entry in self._expected_regex:
            regex, hz_str = entry.split("|")
            expected_regex.append(
                {"regex": regex, "expected_hz": float(hz_str) if hz_str else None}
            )
        manifest = {
            "session": {
                "participant": self._participant_id,
                "started": datetime.now().isoformat(timespec="seconds"),
                "min_rate_factor": self._session_check_min_rate_factor,
            },
            "expected_topics": expected,
            "expected_regex": expected_regex,
            "bag_exclude_regex": self._bag_exclude_regex,
        }
        path = os.path.join(session_dir, "expected_topics.yaml")
        with open(path, "w", encoding="utf-8") as handle:
            yaml.safe_dump(manifest, handle, sort_keys=False)

    def _snapshot_calibration(self, session_dir: str):
        """Copy each sensor's calibration files (intrinsics + depth<->colour
        extrinsics) into the session so recorded raw streams stay
        rectifiable/registrable offline, whatever happens to the rig."""
        for src in self._calib_source_dirs:
            if not os.path.isdir(src):
                self.get_logger().warn(f"Calibration dir missing, not snapshotted: {src}")
                continue
            dst = os.path.join(session_dir, "calibration", os.path.basename(src))
            try:
                shutil.copytree(src, dst, dirs_exist_ok=True)
            except OSError as exc:
                self.get_logger().warn(f"Calibration snapshot failed for {src}: {exc}")

    def _run_session_check(self, session_dir: str) -> str:
        """Run the post-session sanity check; returns a one-line summary."""
        if not self._session_check_enabled or not session_dir:
            return ""
        cmd = ["ros2", "run", "session_recorder", "session_check", session_dir]
        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=120.0
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().error(f"session_check failed to run: {exc}")
            return "session_check: FAILED TO RUN"
        summary = (result.stdout or "").strip().splitlines()
        summary = summary[-1] if summary else ""
        if result.returncode == 0:
            self.get_logger().info(f"session_check PASS: {summary}")
            return f"session_check: PASS ({summary})"
        self.get_logger().error(
            f"session_check FAIL: {summary}\n{result.stdout}\n{result.stderr}"
        )
        return f"session_check: FAIL ({summary}) - see session_report.yaml"

    def _stop_processes(self):
        with self._lock:
            color_proc = self._color_proc
            bag_proc = self._bag_proc
            throttle_proc = self._throttle_proc
            session_dir = self._session_dir
            color_log = self._color_log
            bag_log = self._bag_log
            throttle_log = self._throttle_log
            self._active = False
            self._color_proc = None
            self._bag_proc = None
            self._throttle_proc = None
            self._session_dir = ""
            self._color_log = None
            self._bag_log = None
            self._throttle_log = None

        # Recorders first so they drain cleanly; the throttle relay last so
        # the bag never waits on topics whose upstream has already vanished.
        procs = (color_proc, bag_proc, throttle_proc)
        for proc in procs:
            if proc is None:
                continue
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass

        deadline = time.time() + 30.0
        for proc in procs:
            if proc is None:
                continue
            remaining = max(0.0, deadline - time.time())
            try:
                proc.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass

        for log_file in (color_log, bag_log, throttle_log):
            if log_file is None:
                continue
            try:
                log_file.close()
            except Exception:
                pass

        return session_dir

    def _nexus_send(self, xml_payload: str):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.sendto(xml_payload.encode("utf-8") + b"\x00",
                     (self._nexus_host, self._nexus_port))
        sock.close()

    def _nexus_start(self):
        packet_id = random.randint(0, 999999)
        trial = f"{self._participant_id}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self._nexus_trial_name = trial
        payload = (
            '<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n'
            '<CaptureStart>\n'
            f'  <Name VALUE="{trial}"/>\n'
            '  <Notes VALUE=""/>\n'
            '  <Description VALUE=""/>\n'
            f'  <DatabasePath VALUE="{self._nexus_path}"/>\n'
            '  <Delay VALUE="0"/>\n'
            f'  <PacketID VALUE="{packet_id}"/>\n'
            '</CaptureStart>\n'
        )
        try:
            self._nexus_send(payload)
            self.get_logger().info(
                f"Nexus capture start sent to {self._nexus_host}:{self._nexus_port}"
                f"  trial={trial}  packet_id={packet_id}"
            )
        except OSError as exc:
            self.get_logger().warn(f"Failed to send Nexus start: {exc}")

    def _nexus_stop(self):
        if not self._nexus_trial_name:
            return
        packet_id = random.randint(0, 999999)
        payload = (
            '<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n'
            '<CaptureStop>\n'
            f'  <Name VALUE="{self._nexus_trial_name}"/>\n'
            f'  <DatabasePath VALUE="{self._nexus_path}"/>\n'
            f'  <PacketID VALUE="{packet_id}"/>\n'
            '</CaptureStop>\n'
        )
        try:
            self._nexus_send(payload)
            self.get_logger().info(
                f"Nexus capture stop sent to {self._nexus_host}:{self._nexus_port}"
                f"  trial={self._nexus_trial_name}"
            )
        except OSError as exc:
            self.get_logger().warn(f"Failed to send Nexus stop: {exc}")
        self._nexus_trial_name = ""

    def _on_start(self, request, response):
        del request
        record_video = bool(self._color_topics or self._color_compressed_topics)
        record_bag = bool(self._bag_topics or self._bag_regex)

        with self._lock:
            if self._active:
                response.success = False
                response.message = "Recording already active"
                return response
            if not record_video and not record_bag:
                response.success = False
                response.message = "Nothing to record: no video or bag topics configured"
                return response

            self._session_dir = self._session_path()
            output_dir = os.path.join(self._session_dir, "videos")
            bag_dir = self._unique_bag_dir(os.path.join(self._session_dir, "bag"))
            os.makedirs(output_dir if record_video else self._session_dir, exist_ok=True)

            color_log_path = os.path.join(output_dir, "colour_video_recorder.log")
            bag_log_path = os.path.join(self._session_dir, "rosbag_record.log")
            throttle_log_path = os.path.join(self._session_dir, "topic_throttle.log")

        self._write_manifest(self._session_dir)
        self._snapshot_calibration(self._session_dir)

        try:
            if self._bag_throttles:
                # Start the relays before the bag so the throttled topics
                # exist by the time discovery runs.
                self._throttle_proc, self._throttle_log = self._start_process(
                    self._build_throttle_cmd(),
                    throttle_log_path,
                )
            if record_video:
                self._color_proc, self._color_log = self._start_process(
                    self._build_color_cmd(output_dir),
                    color_log_path,
                )
            if record_bag:
                self._bag_proc, self._bag_log = self._start_process(
                    self._build_bag_cmd(bag_dir),
                    bag_log_path,
                )
            time.sleep(0.2)
            if self._color_proc is not None and self._color_proc.poll() is not None:
                self.get_logger().error(
                    f"Colour recorder exited early. See {color_log_path}"
                )
            if self._bag_proc is not None and self._bag_proc.poll() is not None:
                self.get_logger().error(
                    f"Rosbag recorder exited early. See {bag_log_path}"
                )
        except Exception as exc:
            self.get_logger().error(f"Failed to start recording: {exc}")
            self._stop_processes()
            response.success = False
            response.message = f"Failed to start recording: {exc}"
            return response

        with self._lock:
            self._active = True

        self.get_logger().info(f"Recording started in {self._session_dir}")
        if self._nexus_capture:
            self._nexus_start()
        response.success = True
        response.message = f"Recording started in {self._session_dir}"
        return response

    def _on_stop(self, request, response):
        del request
        with self._lock:
            if not self._active:
                response.success = False
                response.message = "No active recording"
                return response

        session_dir = self._stop_processes()
        if self._nexus_capture:
            self._nexus_stop()
        check_summary = self._run_session_check(session_dir)
        self.get_logger().info(f"Recording stopped: {session_dir}")
        response.success = True
        response.message = f"Recording stopped: {session_dir}"
        if check_summary:
            response.message += f"\n{check_summary}"
        return response

    def destroy_node(self):
        if self._active:
            self._stop_processes()
            if self._nexus_capture:
                self._nexus_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RecordingManager()
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
