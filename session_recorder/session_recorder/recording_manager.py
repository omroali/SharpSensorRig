#!/usr/bin/env python3
"""Unified service-driven recording manager for the whole sensor rig.

One /start_recording -> /stop_recording pair records every sensor marked
record: true in the unified config directory: Kinect + RealSense colour to
H.265 videos, and depth/IR/camera_info/velodyne/vicon/TF into one rosbag.
Each take lands in a single session directory:

  <output_root>/<uuid>/session_<stamp>/
    videos/*.mkv + per-stream timestamp CSVs
    bag/         one mcap with every non-video topic (split every
                 bag_max_duration_s / bag_max_size_mb - see below)

Survivability (2026-10-01 incident)
-----------------------------------
The 2026-10-01 session was killed by a tmux server segfault 3 s into the first
trial. Because the bag was one unbounded ``bag_0.mcap`` with no finalisation,
72 minutes of data became unreadable, all four videos were 0 bytes (NVENC could
not initialise) and no report was ever written. This manager now:

* splits the bag (``--max-bag-size`` / ``--max-bag-duration``) so a crash costs
  at most one split instead of the whole session;
* installs SIGHUP/SIGTERM handlers that run the graceful stop — the only path
  that writes the MCAP footer, ``metadata.yaml`` and ``session_report.yaml`` —
  and forwards SIGINT to the recorder children (which were started with
  ``os.setsid`` and therefore never see the terminal's signal);
* watches the recorder children for the whole session instead of polling once
  0.2 s after launch, restarts the bag recorder into a fresh ``bag_N`` on death,
  and writes ``recorder_heartbeat.json`` every tick;
* echoes the recorder's encoder verdict onto the operator's console (an NVENC
  failure now says so loudly at start rather than producing four 0-byte files).
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

from session_recorder import hardening
from session_recorder.hardening import (
    DEFAULT_BAG_MAX_DURATION_S,
    DEFAULT_BAG_MAX_SIZE_MB,
    DEFAULT_STOP_GRACE_S,
    DEFAULT_VIDEO_STARTUP_GRACE_S,
    DEFAULT_WATCHDOG_INTERVAL_S,
    BAG_STALL_TICKS,
    VIDEO_STALL_TICKS,
    StopRequest,
    append_event_log,
    bag_split_args,
    build_heartbeat,
    install_stop_handlers,
    poll_children,
    read_encoder_status,
    read_stream_failures,
    new_failure_keys,
)
from session_recorder.topics import parse_expected_entry
from session_recorder.video_encoding import resolve_encoder


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
        self.declare_parameter("video_software_fallback", True)
        self.declare_parameter("video_encoder_probe", True)
        self.declare_parameter("video_startup_grace_s", DEFAULT_VIDEO_STARTUP_GRACE_S)
        self.declare_parameter("bag_storage", "mcap")
        self.declare_parameter("bag_compression", "none")
        self.declare_parameter("mcap_preset", "zstd_fast")
        self.declare_parameter("bag_cache_size_mb", 128)
        self.declare_parameter("bag_max_size_mb", DEFAULT_BAG_MAX_SIZE_MB)
        self.declare_parameter("bag_max_duration_s", DEFAULT_BAG_MAX_DURATION_S)
        self.declare_parameter("watchdog_enabled", True)
        self.declare_parameter("watchdog_interval_s", DEFAULT_WATCHDOG_INTERVAL_S)
        self.declare_parameter("watchdog_restart_bag", True)
        self.declare_parameter("watchdog_restart_throttle", True)
        self.declare_parameter("stop_grace_s", DEFAULT_STOP_GRACE_S)
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
        self._video_software_fallback = bool(
            self.get_parameter("video_software_fallback").value
        )
        self._video_encoder_probe = bool(
            self.get_parameter("video_encoder_probe").value
        )
        self._video_startup_grace_s = max(
            0.0, float(self.get_parameter("video_startup_grace_s").value)
        )
        self._bag_storage = str(self.get_parameter("bag_storage").value)
        self._bag_cache_size_mb = int(self.get_parameter("bag_cache_size_mb").value)
        self._bag_max_size_mb = float(self.get_parameter("bag_max_size_mb").value)
        self._bag_max_duration_s = float(
            self.get_parameter("bag_max_duration_s").value
        )
        self._watchdog_enabled = bool(self.get_parameter("watchdog_enabled").value)
        self._watchdog_interval_s = max(
            1.0, float(self.get_parameter("watchdog_interval_s").value)
        )
        self._watchdog_restart_bag = bool(
            self.get_parameter("watchdog_restart_bag").value
        )
        self._watchdog_restart_throttle = bool(
            self.get_parameter("watchdog_restart_throttle").value
        )
        self._stop_grace_s = max(1.0, float(self.get_parameter("stop_grace_s").value))
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

        # Watchdog / heartbeat / alarm state (reset at every start).
        self._watchdog_thread = None
        self._watchdog_stop = threading.Event()
        self._bag_restarts = 0
        self._throttle_restarts = 0
        self._watchdog_events: list[str] = []
        self._reported_dead: set[str] = set()
        self._encoder_status_reported = False
        self._seen_stream_failures: set[str] = set()
        self._video_checked = False
        self._video_stall_ticks = 0
        self._video_stall_reported = False
        self._bag_stall_ticks = 0
        self._bag_stall_reported = False
        self._last_video_bytes = 0
        self._last_bag_bytes = 0
        self._expected_video_topics: list[str] = []
        for entry in self._expected_topics:
            try:
                topic, _hz, sink = parse_expected_entry(entry)
            except ValueError:
                continue
            if sink == "video":
                self._expected_video_topics.append(topic)

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
            f"  bag_split         : max {self._bag_max_size_mb:.0f} MiB / "
            f"{self._bag_max_duration_s:.0f} s\n"
            f"  watchdog          : {self._watchdog_enabled} "
            f"(every {self._watchdog_interval_s:.0f} s, "
            f"restart bag={self._watchdog_restart_bag})\n"
            f"  nexus_capture     : {self._nexus_capture}\n"
            f"  nexus_host        : {self._nexus_host}\n"
            f"  nexus_port        : {self._nexus_port}\n"
            f"  nexus_path        : {self._nexus_path}"
        )

        # Encoder pre-flight (B4) at rig start, not at participant time: the
        # 2026-10-01 rig started with an unusable NVENC and said nothing.
        if self._video_encoder_probe and self._video_encoder:
            decision = resolve_encoder(
                self._video_encoder,
                allow_software_fallback=self._video_software_fallback,
            )
            if decision.ok and not decision.fallback_used:
                self.get_logger().info(f"Encoder pre-flight: {decision.summary()}")
            else:
                self.get_logger().error(f"Encoder pre-flight: {decision.summary()}")

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
            "-p", f"software_fallback:={'true' if self._video_software_fallback else 'false'}",
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
            # Splitting is the single highest-value change from the 2026-10-01
            # incident: each completed bag_N.mcap is finalised with its own
            # footer, so a crash costs one split (~5 min), not the session.
            *bag_split_args(self._bag_max_size_mb, self._bag_max_duration_s),
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
        # The check prints its own summary, but `ros2 run` appends
        # "[ros2run]: Process exited with failure 1" after it -- taking the last
        # line blindly reported that launcher noise instead of the summary, which
        # is the one line the operator reads at the end of a session.
        summary = hardening.session_check_summary(result.stdout or "")
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

        deadline = time.time() + self._stop_grace_s
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
            self._session_start = time.time()
        self._reset_watchdog_state()
        self._ensure_watchdog()

        self.get_logger().info(
            f"Recording started in {self._session_dir}\n"
            f"  bag splitting: max {self._bag_max_size_mb:.0f} MiB / "
            f"{self._bag_max_duration_s:.0f} s per split"
        )
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

        session_dir, check_summary = self.graceful_shutdown("stop_recording service")
        response.success = True
        response.message = f"Recording stopped: {session_dir}"
        if check_summary:
            response.message += f"\n{check_summary}"
        return response

    # ------------------------------------------------------------------ #
    # Graceful stop — the only path that finalises the bag
    # ------------------------------------------------------------------ #
    def graceful_shutdown(self, reason: str) -> tuple[str, str]:
        """Stop the recorders and finalise the session. Idempotent.

        Writes the MCAP footer + ``metadata.yaml`` (rosbag2 does that when its
        SIGINT lands), ``recorder_heartbeat.json`` and ``session_report.yaml``.
        Called by /stop_recording and by the SIGHUP/SIGTERM handler in ``main``;
        ``destroy_node`` keeps a last-resort path that finalises the bag but
        cannot run the session check.
        """
        with self._lock:
            if not self._active:
                return self._session_dir, ""
            session_dir = self._session_dir
        self.get_logger().warn(f"Stopping recording ({reason})")
        self._watchdog_note(f"stopping recording: {reason}", session_dir)
        session_dir = self._stop_processes()
        if self._nexus_capture:
            self._nexus_stop()
        check_summary = self._run_session_check(session_dir)
        self.get_logger().info(f"Recording stopped: {session_dir}")
        if check_summary:
            self.get_logger().info(check_summary)
        return session_dir, check_summary

    def destroy_node(self):
        self._stop_watchdog()
        if self._active:
            self._stop_processes()
            if self._nexus_capture:
                self._nexus_stop()
        super().destroy_node()

    # ------------------------------------------------------------------ #
    # Watchdog (2026-10-01: a recorder that died later was never noticed,
    # logged or restarted — the only check was a single poll() 0.2 s after
    # launch)
    # ------------------------------------------------------------------ #
    def _children(self) -> dict:
        with self._lock:
            return {
                "colour_video_recorder": self._color_proc,
                "rosbag_record": self._bag_proc,
                "topic_throttle": self._throttle_proc,
            }

    def _reset_watchdog_state(self):
        self._bag_restarts = 0
        self._throttle_restarts = 0
        self._watchdog_events = []
        self._reported_dead = set()
        self._encoder_status_reported = False
        self._seen_stream_failures = set()
        self._video_checked = False
        self._video_stall_ticks = 0
        self._video_stall_reported = False
        self._bag_stall_ticks = 0
        self._bag_stall_reported = False
        self._last_video_bytes = 0
        self._last_bag_bytes = 0

    def _ensure_watchdog(self):
        if not self._watchdog_enabled:
            return
        if self._watchdog_thread is not None and self._watchdog_thread.is_alive():
            return
        self._watchdog_stop = threading.Event()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, name="recorder-watchdog", daemon=True
        )
        self._watchdog_thread.start()

    def _stop_watchdog(self):
        if self._watchdog_thread is None:
            return
        self._watchdog_stop.set()
        self._watchdog_thread.join(timeout=2.0)
        self._watchdog_thread = None

    def _watchdog_loop(self):
        while not self._watchdog_stop.wait(self._watchdog_interval_s):
            try:
                self._watchdog_tick()
            except Exception as exc:  # a watchdog must never kill the node
                self.get_logger().error(f"WATCHDOG: tick failed: {exc}")

    def _watchdog_alarm(self, message: str, session_dir: str = ""):
        """Red console line + forensic log line; each episode alarms once."""
        session_dir = session_dir or self._session_dir
        self.get_logger().error(f"WATCHDOG: {message}")
        self._watchdog_events = (self._watchdog_events + [message])[-20:]
        if session_dir:
            append_event_log(hardening.watchdog_log_path(session_dir), f"ALARM {message}")

    def _watchdog_note(self, message: str, session_dir: str = ""):
        session_dir = session_dir or self._session_dir
        if session_dir:
            append_event_log(hardening.watchdog_log_path(session_dir), f"NOTE  {message}")

    def _watchdog_tick(self):
        with self._lock:
            active = self._active
            session_dir = self._session_dir
            started = self._session_start
        if not active or not session_dir:
            return

        statuses = poll_children(self._children())
        dead = [status for status in statuses if not status.alive]
        dead_names = {status.name for status in dead}
        for status in dead:
            if status.name in self._reported_dead:
                continue  # one alarm per death episode, not one per tick
            self._reported_dead.add(status.name)
            self._handle_dead_child(status, session_dir)
        for name in list(self._reported_dead):
            if name not in dead_names:
                self._reported_dead.discard(name)

        self._report_encoder_status(session_dir)
        self._report_stream_failures(session_dir)

        elapsed = time.time() - started if started else 0.0
        video_sizes = hardening.video_file_sizes(session_dir, self._expected_video_topics)
        bag_sizes = hardening.bag_file_sizes(session_dir)
        self._check_video_health(session_dir, elapsed, video_sizes)
        self._check_bag_progress(session_dir, bag_sizes)
        # Re-poll: a restart above must show up as alive in the heartbeat.
        self._write_heartbeat(
            session_dir,
            started,
            poll_children(self._children()),
            video_sizes,
            bag_sizes,
        )

    def _handle_dead_child(self, status, session_dir: str):
        if status.name == "rosbag_record":
            self._watchdog_alarm(
                f"{status.summary()} — the bag is no longer being written", session_dir
            )
            if self._watchdog_restart_bag:
                self._restart_bag_recorder(status.summary(), session_dir)
            else:
                self._watchdog_alarm(
                    "watchdog_restart_bag is false: data from here on is NOT recorded",
                    session_dir,
                )
        elif status.name == "topic_throttle":
            self._watchdog_alarm(f"{status.summary()}", session_dir)
            if self._watchdog_restart_throttle:
                self._restart_throttle(session_dir)
        else:
            self._watchdog_alarm(
                f"{status.summary()} — no video is being written (the bag continues)",
                session_dir,
            )

    def _restart_bag_recorder(self, reason: str, session_dir: str):
        with self._lock:
            if not self._active:
                return
            old_log = self._bag_log
            # _unique_bag_dir keeps every earlier split intact: a restarted bag
            # lands in bag_1/, bag_2/, ... and session_check aggregates them all.
            bag_dir = self._unique_bag_dir(os.path.join(session_dir, "bag"))
            index = self._bag_restarts + 1
            log_path = os.path.join(session_dir, f"rosbag_record_restart_{index}.log")
        if old_log is not None:
            try:
                old_log.close()
            except Exception:
                pass
        try:
            proc, log_file = self._start_process(
                self._build_bag_cmd(bag_dir), log_path
            )
        except Exception as exc:
            self._watchdog_alarm(
                f"failed to restart the bag recorder: {exc} — RECORDING LOST FROM HERE",
                session_dir,
            )
            return
        with self._lock:
            self._bag_proc = proc
            self._bag_log = log_file
            self._bag_restarts += 1
        self._watchdog_alarm(
            f"restarted the bag recorder into {os.path.basename(bag_dir)}/ "
            f"({reason}); log: {os.path.basename(log_path)}",
            session_dir,
        )

    def _restart_throttle(self, session_dir: str):
        with self._lock:
            if not self._active:
                return
            old_log = self._throttle_log
            index = self._throttle_restarts + 1
            log_path = os.path.join(session_dir, f"topic_throttle_restart_{index}.log")
        if old_log is not None:
            try:
                old_log.close()
            except Exception:
                pass
        try:
            proc, log_file = self._start_process(self._build_throttle_cmd(), log_path)
        except Exception as exc:
            self._watchdog_alarm(f"failed to restart topic_throttle: {exc}", session_dir)
            return
        with self._lock:
            self._throttle_proc = proc
            self._throttle_log = log_file
            self._throttle_restarts += 1
        self._watchdog_alarm("restarted the topic_throttle relay", session_dir)

    def _report_encoder_status(self, session_dir: str):
        """Echo the recorder's encoder verdict onto the operator's console."""
        if self._encoder_status_reported:
            return
        status = read_encoder_status(session_dir)
        if not status:
            return
        self._encoder_status_reported = True
        if status.get("fallback_used"):
            self._watchdog_alarm(
                "VIDEO ENCODER FALLBACK ACTIVE: NVENC could not initialise; "
                f"encoding with {status.get('effective_encoders')} on the CPU. "
                "Video is still being recorded.",
                session_dir,
            )
        broken = list(status.get("broken_streams") or [])
        if broken:
            self._watchdog_alarm(
                f"NO WORKING ENCODER for {broken}: those videos WILL be empty",
                session_dir,
            )

    def _report_stream_failures(self, session_dir: str):
        """Surface recorder-side stream deaths (the 4x0-byte failure mode)."""
        failures = read_stream_failures(session_dir)
        for key in new_failure_keys(self._seen_stream_failures, failures):
            self._seen_stream_failures.add(key)
            entry = failures.get(key) or {}
            reason = str(entry.get("reason", "unknown failure"))
            if str(entry.get("severity", "error")) == "error":
                self._watchdog_alarm(f"STREAM FAILURE [{key}]: {reason}", session_dir)
            else:
                self.get_logger().warn(f"WATCHDOG: stream note [{key}]: {reason}")
                self._watchdog_note(f"stream note [{key}]: {reason}", session_dir)

    def _check_video_health(self, session_dir: str, elapsed: float, sizes: dict):
        if not self._expected_video_topics:
            return
        total = sum(size for size in sizes.values() if size > 0)
        past_grace = elapsed >= self._video_startup_grace_s
        if past_grace and not self._video_checked:
            self._video_checked = True
            problems = hardening.video_startup_problems(
                session_dir, self._expected_video_topics
            )
            if problems:
                for problem in problems:
                    self._watchdog_alarm(f"VIDEO CHECK FAILED: {problem}", session_dir)
            else:
                self.get_logger().info(
                    f"WATCHDOG: video check OK - {len(self._expected_video_topics)} "
                    "stream(s) writing non-empty files"
                )
        if total > self._last_video_bytes:
            self._last_video_bytes = total
            self._video_stall_ticks = 0
            self._video_stall_reported = False
        elif past_grace:
            self._video_stall_ticks += 1
            if (
                self._video_stall_ticks >= VIDEO_STALL_TICKS
                and not self._video_stall_reported
            ):
                self._video_stall_reported = True
                self._watchdog_alarm(
                    "video files have not grown for "
                    f"{self._video_stall_ticks * self._watchdog_interval_s:.0f}s "
                    f"({total} bytes total) - video recording has stopped",
                    session_dir,
                )

    def _check_bag_progress(self, session_dir: str, sizes: dict):
        total = sum(size for size in sizes.values() if size > 0)
        if total > self._last_bag_bytes:
            self._last_bag_bytes = total
            self._bag_stall_ticks = 0
            self._bag_stall_reported = False
            return
        self._bag_stall_ticks += 1
        # Warning, not error: the rosbag2 write cache can legitimately hold
        # writes for ~1 minute, so this is deliberately conservative.
        if self._bag_stall_ticks >= BAG_STALL_TICKS and not self._bag_stall_reported:
            self._bag_stall_reported = True
            self.get_logger().warn(
                "WATCHDOG: no bag growth for "
                f"{self._bag_stall_ticks * self._watchdog_interval_s:.0f}s "
                f"({total} bytes); if this persists the bag recorder is wedged"
            )
            self._watchdog_note("bag growth stalled", session_dir)

    def _write_heartbeat(self, session_dir: str, started: float, statuses, sizes: dict, bag_sizes: dict):
        payload = build_heartbeat(
            session_dir=session_dir,
            participant=self._participant_id,
            active=True,
            started_at=started or None,
            children={status.name: status for status in statuses},
            bag_restarts=self._bag_restarts,
            watchdog_events=self._watchdog_events,
            video_sizes=sizes,
            bag_sizes=bag_sizes,
        )
        try:
            hardening.write_atomic_json(
                hardening.heartbeat_path(session_dir), payload
            )
        except OSError as exc:
            self.get_logger().warn(f"could not write recorder heartbeat: {exc}")


def main(args=None):
    rclpy.init(args=args)
    # 2026-10-01: the tmux server segfaulted, the pane's SIGHUP killed this
    # process, and 72 minutes of mcap were left with no footer, no metadata.yaml
    # and no session_report.yaml. SIGHUP (dying terminal) and SIGTERM
    # (supervisor / `docker stop`) now request the same graceful stop as
    # /stop_recording. The handler only sets a flag; the teardown runs here on
    # the main thread. Installed before node construction so the encoder
    # pre-flight probe is covered too.
    stop = StopRequest()
    install_stop_handlers(stop)
    node = RecordingManager()
    try:
        while rclpy.ok() and not stop.requested:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted (SIGINT)")
    except hardening.ros_shutdown_exceptions():
        node.get_logger().warn("ROS context shut down externally")
    finally:
        reason = stop.reason() if stop.requested else "interrupt or node shutdown"
        try:
            node.graceful_shutdown(reason)
        except Exception as exc:  # a failed teardown must still be visible
            node.get_logger().error(f"graceful shutdown failed: {exc}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
