#!/usr/bin/env python3
"""Live topic-health web dashboard for the sensor rig.

Subscribes to every topic the rig is configured to record (derived from the
same configs as the recorder, via topics.build_recording_setup) and serves a
small dark web UI showing each topic's measured Hz, an expected-rate status
pill, a rolling sparkline, plus disk headroom and live bag growth.

Subscriptions use raw serialized buffers (raw=True) and best-effort QoS: no
message is ever deserialized, so the per-message cost is a timestamp append
regardless of payload size. Safe to leave running alongside recording.

Runs independently of capture sessions; launch it once and leave it up:
  ros2 run session_recorder topic_health_monitor
Config comes from recording.yaml `monitor:` and the sensor YAMLs in
$SENSOR_CONFIG_DIR.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import threading
import time
import urllib.error
import urllib.request
from collections import deque

import rclpy
import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from std_srvs.srv import Trigger

from session_recorder import hardening
from session_recorder.topics import (
    build_recording_setup,
    config_dir,
    load_yaml,
    parse_expected_entry,
)

DISCOVERY_PERIOD_S = 2.0
# History kept per topic for the sparkline (one point per UI poll).
SPARK_POINTS = 40

#: Recording control surface (recording_manager, an ``std_srvs/Trigger`` pair).
START_SERVICE = "/start_recording"
STOP_SERVICE = "/stop_recording"

#: How often the monitor refreshes its view of which services exist. The ROS
#: graph query is not free, so it is cached rather than asked per web poll.
SERVICE_POLL_S = 2.0

#: The recording manager rewrites ``recorder_heartbeat.json`` once per watchdog
#: tick (every 5 s, ``watchdog.interval_s``). A heartbeat file not written
#: within this many seconds means the manager has died or stopped, and the UI
#: must stop claiming the rig is recording. Without this a crashed rig would
#: show a healthy glowing REC light forever.
HEARTBEAT_STALE_S = 10.0

#: Audible confirmation that a recording started (one beep) or stopped (three).
#:
#: This container has no audio device, so the monitor cannot play anything
#: itself; it asks a small host listener (``SHARP/rig_cue_server.py``) to. The
#: rig runs with ``network_mode: host``, so loopback here is the host's loopback
#: and no port mapping is needed. Empty disables the cues.
#:
#: Deliberately best-effort: a rig with no speakers, or with the listener not
#: running, must still be able to start and stop recordings.
CUE_SERVER_URL = os.environ.get("SHARP_CUE_URL", "http://127.0.0.1:8766")
CUE_TIMEOUT_S = 3.0

#: Host helper that (re)starts the cue listener if it is not already up. The
#: container sees the host's /usr/local/bin read-only, so this is installed
#: there. Invoking it before a cue is what makes the feature self-healing:
#: relying on the operator to have started the listener is precisely how the
#: cues ended up silently doing nothing, with no indication why.
CUE_ENSURE_BIN = os.environ.get("SHARP_CUE_ENSURE", "sharp-cue-ensure")


def topic_group(topic: str) -> str:
    """Human-friendly grouping label for the UI (sensor the topic belongs to)."""
    parts = topic.strip("/").split("/")
    if not parts or not parts[0]:
        return "misc"
    head = parts[0]
    if head == "realsense" and len(parts) > 1:
        return parts[1]
    if head in ("tf", "tf_static"):
        return "tf"
    return head


class TopicStat:
    def __init__(self, topic: str, expected_hz: float | None, sink: str, group: str):
        self.topic = topic
        self.expected_hz = expected_hz
        self.sink = sink
        self.group = group
        self.stamps: deque[float] = deque(maxlen=4096)
        self.connected = False
        self.spark: deque[float] = deque([0.0] * SPARK_POINTS, maxlen=SPARK_POINTS)

    def record(self):
        self.stamps.append(time.monotonic())

    def hz(self, window_s: float) -> float:
        now = time.monotonic()
        cutoff = now - window_s
        while self.stamps and self.stamps[0] < cutoff:
            self.stamps.popleft()
        if len(self.stamps) < 2:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        return (len(self.stamps) - 1) / span if span > 0 else 0.0

    def status(self, window_s: float, min_rate_factor: float) -> tuple[str, float]:
        hz = self.hz(window_s)
        self.spark.append(hz)
        if not self.connected:
            return "waiting", hz
        if hz <= 0.0:
            return "silent", hz
        if self.expected_hz is None:
            return "ok", hz
        return ("ok" if hz >= self.expected_hz * min_rate_factor else "degraded"), hz


class TopicHealthMonitor(Node):
    def __init__(self):
        super().__init__("topic_health_monitor")
        self.declare_parameter("window_seconds", 5.0)
        self.declare_parameter("min_rate_factor", 0.5)
        self.declare_parameter("output_root", os.path.expanduser("~/data"))
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8765)
        self._window = float(self.get_parameter("window_seconds").value)
        self._min_rate_factor = float(self.get_parameter("min_rate_factor").value)
        self._output_root = str(self.get_parameter("output_root").value)
        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)

        self._lock = threading.Lock()
        self._stats: dict[str, TopicStat] = {}
        self._regex: list[tuple[re.Pattern, float | None]] = []
        self._exclude: re.Pattern | None = None

        # Cached ROS graph view for the recording control surface. Refreshed by
        # a timer, never per web request.
        #
        # NOTE the ``control_`` prefix is load-bearing. rclpy's Node already
        # keeps ``self._services`` as its list of service *entities*; rclpy.spin
        # iterates it and ``destroy_node`` does ``self._services[0]``. Shadowing
        # that name with a set of strings crashed the node inside spin() with
        # "'str' object has no attribute '_executor_event'". Never name this
        # attribute ``_services``.
        self._control_services: set[str] = set()
        self._last_command: dict = {}

        self._load_targets()

        self._discovery_timer = self.create_timer(
            DISCOVERY_PERIOD_S, self._discover
        )
        self._service_timer = self.create_timer(
            SERVICE_POLL_S, self._refresh_services
        )
        self._discover()
        self._refresh_services()

    # ---- recording control surface ----------------------------------------

    def _refresh_services(self):
        """Cache which control services are currently advertised.

        ``get_service_names_and_types()`` yields ``(name, [types])`` pairs, not
        bare names -- collecting the pairs into a set silently makes every
        membership test false, which shows the operator "service not running"
        over a service that is in fact up.
        """
        try:
            entries = self.get_service_names_and_types()
        except Exception:  # noqa: BLE001 - a dead graph must not kill the UI
            self._control_services = set()
            return
        names = set()
        for entry in entries:
            # Accept both the documented pair form and a bare string, so a
            # future rclpy that flattens this cannot break the panel again.
            if isinstance(entry, str):
                names.add(entry)
            else:
                try:
                    names.add(str(entry[0]))
                except (TypeError, IndexError, KeyError):
                    continue
        self._control_services = names

    def _newest_session_dir(self) -> str:
        """Most recently modified session directory under ``output_root``.

        Shallow by design: ``output_root/<participant>/session_*/``, stat only.
        This is what ``recording_manager._unique_bag_dir`` feeds, so the newest
        directory is the one a live recording is writing into.
        """
        root = self._output_root
        if not os.path.isdir(root):
            return ""
        newest, newest_mtime = "", -1.0
        try:
            participants = list(os.scandir(root))
        except OSError:
            return ""
        for participant in participants:
            try:
                if not participant.is_dir():
                    continue
                with os.scandir(participant.path) as sessions:
                    for session in sessions:
                        try:
                            if session.is_dir() and session.stat().st_mtime > newest_mtime:
                                newest_mtime = session.stat().st_mtime
                                newest = session.path
                        except OSError:
                            continue
            except OSError:
                # e.g. root-owned lost+found on a freshly mounted drive
                continue
        return newest

    def recording_info(self) -> dict:
        """Everything the dashboard needs to show and drive a recording.

        Reads the recording manager's ``recorder_heartbeat.json`` rather than
        walking the data directory: the manager already publishes participant,
        session, elapsed time and byte counts there every few hundred ms, so
        this is one small file read plus a shallow stat scan.
        """
        services_up = (
            START_SERVICE in self._control_services
            and STOP_SERVICE in self._control_services
        )
        info = {
            "service_up": services_up,
            "active": False,
            "stale": False,
            "participant": "",
            "session": "",
            "session_dir": "",
            "elapsed_s": 0.0,
            "video_gb": 0.0,
            "bag_gb": 0.0,
            "children": {},
            "last_command": dict(self._last_command),
        }
        if not services_up:
            return info

        session_dir = self._newest_session_dir()
        if not session_dir:
            return info
        heartbeat = hardening.read_json(hardening.heartbeat_path(session_dir))
        if not heartbeat:
            return info

        info.update({
            "participant": str(heartbeat.get("participant") or ""),
            "session": os.path.basename(session_dir),
            "session_dir": session_dir,
            "children": dict(heartbeat.get("children") or {}),
            "video_gb": round(
                sum(heartbeat.get("video_bytes", {}).values()) / (1024 ** 3), 2
            ),
            "bag_gb": round(
                sum(heartbeat.get("bag_bytes", {}).values()) / (1024 ** 3), 2
            ),
        })

        # Liveness comes from the heartbeat's *mtime*, not its ``active`` flag.
        # The manager's final write during a clean stop still carries
        # active=True (it is built before the flag is cleared), so trusting the
        # flag would leave the dashboard showing a glowing REC light forever
        # after a stop. A heartbeat that stopped being rewritten is the honest
        # signal. The cost is a short false-positive window: the watchdog ticks
        # every 5 s, so a just-stopped session can read active for up to
        # HEARTBEAT_STALE_S.
        try:
            age = time.time() - os.path.getmtime(hardening.heartbeat_path(session_dir))
        except OSError:
            age = float("inf")
        info["stale"] = age > HEARTBEAT_STALE_S
        info["active"] = bool(heartbeat.get("active")) and not info["stale"]

        # Count elapsed from wall clock so the browser's timer ticks smoothly
        # between heartbeat writes (which are seconds apart).
        started_at = float(heartbeat.get("started_at") or 0.0)
        if info["active"] and started_at > 0:
            info["elapsed_s"] = round(max(0.0, time.time() - started_at), 1)
        else:
            info["elapsed_s"] = round(float(heartbeat.get("elapsed_s") or 0.0), 1)
        return info

    def _ensure_cue_server(self) -> tuple[bool, str]:
        """Make sure the host cue listener is up. Never raises.

        Returns (reachable, detail). Tries to start the listener through the
        host helper first; if that is not installed, falls back to probing the
        URL so a manually started listener still works.
        """
        if not CUE_SERVER_URL:
            return False, "cues disabled"
        if shutil.which(CUE_ENSURE_BIN):
            try:
                result = subprocess.run(
                    [CUE_ENSURE_BIN], capture_output=True, text=True, timeout=10.0
                )
                payload = json.loads(result.stdout or "{}")
                if payload.get("ok"):
                    return True, str(payload.get("state") or "")
                return False, str(payload.get("detail") or "listener unavailable")
            except subprocess.TimeoutExpired:
                return False, "cue helper timed out"
            except (OSError, ValueError) as exc:
                return False, f"cue helper failed: {exc}"
        # No helper installed: the listener may still be running by hand.
        return self._cue_probe()

    def _cue_probe(self) -> tuple[bool, str]:
        try:
            with urllib.request.urlopen(
                f"{CUE_SERVER_URL.rstrip('/')}/health", timeout=CUE_TIMEOUT_S
            ) as response:
                return response.status == 200, ""
        except urllib.error.URLError as exc:
            return False, f"cue server unreachable ({exc.reason})"
        except Exception as exc:  # noqa: BLE001
            return False, f"cue probe failed: {exc}"

    def _cue(self, kind: str) -> tuple[bool, str]:
        """Ask the host to play the start/stop cue. Never raises.

        Best-effort by design: the operator must be able to start and stop a
        recording on a rig with no speakers, or with the cue listener not
        running, and neither case may be reported as a recording failure.
        """
        if not CUE_SERVER_URL:
            return False, "cues disabled"
        reachable, detail = self._ensure_cue_server()
        if not reachable:
            return False, detail
        url = f"{CUE_SERVER_URL.rstrip('/')}/cue/{kind}"
        try:
            request = urllib.request.Request(url, method="POST", data=b"")
            with urllib.request.urlopen(request, timeout=CUE_TIMEOUT_S) as response:
                payload = json.loads(response.read() or b"{}")
            return bool(payload.get("ok")), str(payload.get("message") or "")
        except urllib.error.URLError as exc:
            return False, f"cue server unreachable ({exc.reason})"
        except Exception as exc:  # noqa: BLE001 - a cue must never break control
            return False, f"cue failed: {exc}"

    def _call_trigger(self, service: str) -> dict:
        """Call a Trigger service on a private node and return {ok, message}.

        Runs on its own node with its own executor: the monitor's node is
        already being spun by rclpy, and adding a service client to it from
        another thread would need the two executors to agree on callback
        groups. A throwaway node keeps that entirely out of the monitor's way.

        Note the caller must not block a web request on this. A stop finalises
        the bag and honours the manager's ``stop_grace_s`` (30 s by default),
        so the HTTP layer fires this in a background thread and polls instead.
        """
        node = None
        try:
            node = Node(f"health_control_{service.strip('/')}")
            client = node.create_client(
                Trigger, service, callback_group=ReentrantCallbackGroup()
            )
            if not client.wait_for_service(timeout_sec=3.0):
                return {"ok": False, "message": f"{service} not available"}
            future = client.call_async(Trigger.Request())
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            deadline = time.time() + 60.0
            while not future.done() and time.time() < deadline:
                executor.spin_once(timeout_sec=0.1)
            executor.remove_node(node)
            if not future.done():
                return {"ok": False, "message": f"{service} timed out"}
            result = future.result()
            return {
                "ok": bool(getattr(result, "success", False)),
                "message": str(getattr(result, "message", "") or ""),
            }
        except Exception as exc:  # noqa: BLE001 - report, never raise into the UI
            return {"ok": False, "message": f"{service} failed: {exc}"}
        finally:
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:  # noqa: BLE001
                    pass

    def request_recording(self, action: str) -> dict:
        """Launch start/stop in the background; return immediately.

        The UI polls ``/api/recording`` for the outcome, so a 30 s bag
        finalisation never holds a browser request open.
        """
        service = START_SERVICE if action == "start" else STOP_SERVICE
        self._last_command = {
            "action": action, "state": "running", "message": "",
            "at": time.time(),
        }

        def run():
            result = self._call_trigger(service)
            message = result["message"]
            cue_note = ""
            # Beep only on a real transition. A failed start, or a stop that
            # errored, must stay silent -- an audible "recording" cue over a
            # recording that never began is worse than no cue at all.
            if result["ok"]:
                played, detail = self._cue(action)
                if not played:
                    cue_note = f" (cue not played: {detail})"
            self._last_command = {
                "action": action,
                "state": "ok" if result["ok"] else "failed",
                "message": message + cue_note,
                "at": time.time(),
            }

        threading.Thread(target=run, daemon=True, name=f"rec-{action}").start()
        return dict(self._last_command)

    # ---- target derivation -------------------------------------------------

    def _load_targets(self):
        cfg_dir = config_dir()
        kinect_cfg = load_yaml(os.path.join(cfg_dir, "kinect_cameras.yaml"))
        realsense_cfg = load_yaml(os.path.join(cfg_dir, "realsense_cameras.yaml"))
        velodyne_cfg = load_yaml(os.path.join(cfg_dir, "velodyne.yaml"))
        recording_raw = load_yaml(os.path.join(cfg_dir, "recording.yaml"))
        recording_settings = (recording_raw or {}).get("recording_settings", {}) or {}

        try:
            setup = build_recording_setup(
                kinect_cfg, realsense_cfg, velodyne_cfg, recording_settings
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to derive monitor targets: {exc}")
            return

        # Throttled bag topics only exist while recording; monitor the source
        # instead so sensor health shows even when idle.
        throttle_src = {}
        for spec in setup.get("bag_throttles", []):
            try:
                src, out, _ = spec.split(":")
                throttle_src[out] = src
            except ValueError:
                continue

        for entry in setup.get("expected_topics", []):
            topic, hz, sink = parse_expected_entry(entry)
            topic = throttle_src.get(topic, topic)
            if topic not in self._stats:
                self._stats[topic] = TopicStat(topic, hz, sink, topic_group(topic))

        for entry in setup.get("expected_regex", []):
            regex, hz_str = entry.split("|")
            self._regex.append(
                (re.compile(regex), float(hz_str) if hz_str else None)
            )
        exclude = setup.get("bag_exclude_regex") or ""
        if exclude:
            self._exclude = re.compile(exclude)

        self.get_logger().info(
            f"Monitoring {len(self._stats)} topics + {len(self._regex)} regex "
            f"patterns; window={self._window}s"
        )

    # ---- ROS graph discovery ----------------------------------------------

    def _discover(self):
        available = dict(self.get_topic_names_and_types())

        # Expand regex patterns against the live graph.
        for topic, types in available.items():
            if topic in self._stats:
                continue
            if self._exclude and self._exclude.fullmatch(topic):
                continue
            for pattern, hz in self._regex:
                if pattern.fullmatch(topic):
                    with self._lock:
                        self._stats[topic] = TopicStat(
                            topic, hz, "bag", topic_group(topic)
                        )
                    break

        for topic, stat in list(self._stats.items()):
            if stat.connected:
                continue
            types = available.get(topic)
            if not types:
                continue
            try:
                self._subscribe(stat, types[0])
            except Exception as exc:
                self.get_logger().warn(f"Cannot subscribe {topic} ({types[0]}): {exc}")

    def _subscribe(self, stat: TopicStat, type_name: str):
        msg_type = get_message(type_name)
        stat.msg_type_name = type_name
        self.create_subscription(
            msg_type,
            stat.topic,
            lambda _msg, s=stat: s.record(),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            raw=True,
        )
        stat.connected = True
        self.get_logger().info(f"Subscribed (monitor) {stat.topic} [{type_name}]")

    # ---- snapshot for the web layer ---------------------------------------

    def snapshot(self) -> dict:
        with self._lock:
            stats = list(self._stats.values())
        groups: dict[str, list] = {}
        n_ok = n_degraded = n_silent = n_waiting = 0
        for stat in sorted(stats, key=lambda s: (s.group, s.topic)):
            status, hz = stat.status(self._window, self._min_rate_factor)
            n_ok += status == "ok"
            n_degraded += status == "degraded"
            n_silent += status == "silent"
            n_waiting += status == "waiting"
            groups.setdefault(stat.group, []).append({
                "topic": stat.topic,
                "hz": round(hz, 1),
                "expected_hz": stat.expected_hz,
                "sink": stat.sink,
                "status": status,
                "spark": [round(v, 1) for v in stat.spark],
            })

        disk = self._disk_info()
        return {
            "time": time.time(),
            "window_s": self._window,
            "summary": {
                "ok": n_ok, "degraded": n_degraded,
                "silent": n_silent, "waiting": n_waiting,
                "total": len(stats),
            },
            "disk": disk,
            "session": self._session_info(),
            "groups": [
                {"name": name, "topics": topics}
                for name, topics in sorted(groups.items())
            ],
        }

    def _disk_info(self) -> dict:
        path = self._output_root if os.path.isdir(self._output_root) else "/"
        try:
            usage = shutil.disk_usage(path)
        except OSError:
            return {}
        gb = 1024 ** 3
        return {
            "path": path,
            "total_gb": round(usage.total / gb, 1),
            "free_gb": round(usage.free / gb, 1),
            "used_pct": round(100.0 * usage.used / usage.total, 1) if usage.total else 0,
        }

    def _session_info(self) -> dict:
        """Newest session dir under output_root and its current bag size."""
        root = self._output_root
        if not os.path.isdir(root):
            return {}
        newest = None
        newest_mtime = -1.0
        try:
            participants = list(os.scandir(root))
        except OSError:
            return {}
        for participant in participants:
            try:
                if not participant.is_dir():
                    continue
                sessions = os.scandir(participant.path)
            except OSError:
                # e.g. root-owned lost+found on a freshly mounted drive
                continue
            with sessions:
                for session in sessions:
                    try:
                        if session.is_dir() and session.stat().st_mtime > newest_mtime:
                            newest_mtime = session.stat().st_mtime
                            newest = session.path
                    except OSError:
                        continue
        if not newest:
            return {}
        bag_bytes = 0
        for base in (os.path.join(newest, "bag"),):
            for dirpath, _dirs, files in os.walk(base):
                for name in files:
                    try:
                        bag_bytes += os.path.getsize(os.path.join(dirpath, name))
                    except OSError:
                        pass
        return {
            "name": os.path.relpath(newest, root),
            "bag_gb": round(bag_bytes / (1024 ** 3), 2),
            "age_s": round(time.time() - newest_mtime, 0),
        }


# --- web layer -------------------------------------------------------------

INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>Rig topic health</title>
<style>
  :root {
    --bg:#0d1117; --panel:#161b22; --border:#30363d; --text:#e6edf3;
    --muted:#8b949e; --ok:#3fb950; --degraded:#d29922; --silent:#f85149;
    --waiting:#6e7681; --accent:#58a6ff;
  }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--text);
    font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;
    font-variant-numeric:tabular-nums; }
  header { padding:16px 24px; border-bottom:1px solid var(--border);
    display:flex; align-items:baseline; gap:16px; flex-wrap:wrap; }
  h1 { font-size:16px; font-weight:600; margin:0; letter-spacing:.2px; }
  .clock { color:var(--muted); font-size:12px; }
  .tiles { display:flex; gap:12px; flex-wrap:wrap; padding:16px 24px; }
  .tile { background:var(--panel); border:1px solid var(--border);
    border-radius:8px; padding:12px 16px; min-width:120px; }
  .tile .label { color:var(--muted); font-size:11px; text-transform:uppercase;
    letter-spacing:.5px; }
  .tile .value { font-size:22px; font-weight:600; margin-top:4px; }
  .tile .sub { color:var(--muted); font-size:11px; margin-top:2px; }
  .dot { display:inline-block; width:9px; height:9px; border-radius:50%;
    margin-right:6px; vertical-align:middle; }
  .s-ok{background:var(--ok);} .s-degraded{background:var(--degraded);}
  .s-silent{background:var(--silent);} .s-waiting{background:var(--waiting);}
  main { padding:0 24px 40px; }
  .group { margin-top:20px; }
  .group h2 { font-size:12px; text-transform:uppercase; letter-spacing:.6px;
    color:var(--muted); margin:0 0 8px; }
  table { width:100%; border-collapse:collapse; background:var(--panel);
    border:1px solid var(--border); border-radius:8px; overflow:hidden; }
  th,td { text-align:left; padding:8px 12px; font-size:13px;
    border-bottom:1px solid var(--border); }
  th { color:var(--muted); font-weight:500; font-size:11px;
    text-transform:uppercase; letter-spacing:.4px; }
  tr:last-child td { border-bottom:none; }
  td.topic { font-family:ui-monospace,SFMono-Regular,Menlo,monospace;
    font-size:12px; }
  td.num { text-align:right; font-variant-numeric:tabular-nums; }
  .pill { display:inline-flex; align-items:center; padding:2px 8px;
    border-radius:999px; font-size:11px; font-weight:600; }
  .pill-ok{color:var(--ok);} .pill-degraded{color:var(--degraded);}
  .pill-silent{color:var(--silent);} .pill-waiting{color:var(--waiting);}
  .badge { font-size:10px; color:var(--muted); border:1px solid var(--border);
    border-radius:4px; padding:1px 5px; margin-left:6px; }
  svg.spark { vertical-align:middle; }
  .muted { color:var(--muted); }

  /* ---- recording control panel ---- */
  .recpanel { margin:16px 24px 0; padding:14px 18px; background:var(--panel);
    border:1px solid var(--border); border-radius:8px;
    display:flex; align-items:center; gap:18px; flex-wrap:wrap; }
  .recpanel.is-recording { border-color:var(--silent);
    box-shadow:0 0 0 1px rgba(248,81,73,.35), 0 0 18px -6px var(--silent); }
  .reclamp { width:14px; height:14px; border-radius:50%; flex:0 0 auto;
    background:var(--waiting); }
  /* Idle = steady grey. Recording = pulsing red. The animation is the whole
     point: it must be obvious from across the room that a capture is running. */
  .recpanel.is-recording .reclamp { background:var(--silent);
    animation:recpulse 1.6s ease-in-out infinite; }
  @keyframes recpulse {
    0%,100% { opacity:1;   box-shadow:0 0 0 0 rgba(248,81,73,.55); }
    50%     { opacity:.35; box-shadow:0 0 0 9px rgba(248,81,73,0); }
  }
  @media (prefers-reduced-motion: reduce) {
    .recpanel.is-recording .reclamp { animation:none; opacity:1; }
  }
  .recstate { font-size:15px; font-weight:600; letter-spacing:.3px; }
  .recmeta { display:flex; gap:18px; flex-wrap:wrap; font-size:12px; }
  .recmeta .k { color:var(--muted); font-size:10px; text-transform:uppercase;
    letter-spacing:.5px; display:block; }
  .recmeta .v { font-size:13px; font-variant-numeric:tabular-nums; }
  .recmeta code { font-family:ui-monospace,SFMono-Regular,Menlo,monospace; }
  .recbtns { margin-left:auto; display:flex; gap:8px; align-items:center; }
  button.rec { font:inherit; font-size:13px; font-weight:600; cursor:pointer;
    padding:8px 16px; border-radius:6px; border:1px solid var(--border);
    background:#21262d; color:var(--text); }
  button.rec:hover:not(:disabled) { filter:brightness(1.25); }
  button.rec:disabled { opacity:.4; cursor:not-allowed; }
  button.rec.go { background:#1f6f3f; border-color:#2ea043; color:#fff; }
  button.rec.halt { background:#8b2c2c; border-color:var(--silent); color:#fff; }
  .recnote { font-size:11px; color:var(--muted); width:100%; }
  .recnote.warn { color:var(--degraded); }
</style>
</head>
<body>
<header>
  <h1>Sensor rig · topic health</h1>
  <span class="clock" id="clock">connecting…</span>
</header>
<div class="recpanel" id="recpanel">
  <span class="reclamp" id="reclamp"></span>
  <div>
    <div class="recstate" id="recstate">Connecting…</div>
    <div class="recnote" id="recnote"></div>
  </div>
  <div class="recmeta" id="recmeta"></div>
  <div class="recbtns">
    <button class="rec go"   id="btn-start" onclick="cmd('start')">● Start recording</button>
    <button class="rec halt" id="btn-stop"  onclick="cmd('stop')">■ Stop &amp; save</button>
  </div>
</div>
<div class="tiles" id="tiles"></div>
<main id="main"></main>
<script>
const SPARK_W = 80, SPARK_H = 20;
function sparkPath(vals){
  if(!vals || vals.length < 2) return "";
  const max = Math.max(1, ...vals);
  const step = SPARK_W/(vals.length-1);
  return vals.map((v,i)=>{
    const x=(i*step).toFixed(1);
    const y=(SPARK_H-(v/max)*SPARK_H).toFixed(1);
    return (i?"L":"M")+x+" "+y;
  }).join(" ");
}
function tile(label,value,sub){
  return `<div class="tile"><div class="label">${label}</div>
    <div class="value">${value}</div><div class="sub">${sub||""}</div></div>`;
}
function fmtDur(s){
  s = Math.max(0, Math.floor(s));
  const h=Math.floor(s/3600), m=Math.floor(s%3600/60), sec=s%60;
  const pad=n=>String(n).padStart(2,"0");
  return (h?h+":"+pad(m):m)+":"+pad(sec);
}
function esc(t){
  return String(t==null?"":t).replace(/[&<>"']/g,
    c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]));
}

// ---- recording control -------------------------------------------------
// Rebuilt only when the *shape* of the state changes; the ticking timer is
// patched in place so a rebuild can never swallow a click on Start/Stop.
let recSig = null, recState = null;

async function cmd(action){
  const label = action==="start" ? "Start recording?" : "Stop and save this session?";
  if(!confirm(label)) return;
  const btns=[document.getElementById("btn-start"),document.getElementById("btn-stop")];
  btns.forEach(b=>b.disabled=true);
  try{
    const r = await fetch("/api/recording/"+action,{method:"POST"});
    const body = await r.json().catch(()=>({}));
    if(!r.ok) alert((body&&body.message)||("HTTP "+r.status));
  }catch(e){ alert("request failed: "+e); }
  // Leave the buttons disabled until the next poll reflects the new state;
  // a start takes seconds and a stop can take the full stop_grace_s.
  pollRecording();
}

function renderRecording(d){
  const panel=document.getElementById("recpanel");
  const sig=[d.service_up,d.active,d.stale,d.participant,d.session,
             JSON.stringify(d.last_command||{})].join("|");
  if(sig===recSig) return;             // nothing structural changed
  recSig=sig;

  panel.classList.toggle("is-recording", !!d.active);
  let state, note="", noteWarn=false;
  if(!d.service_up){
    state = "Recording service not running";
    note = "Start it with: ros2 launch session_recorder "
         + "unified_recording.launch.py  (alias launch_recording)";
    noteWarn = true;
  } else if(d.active){
    state = "RECORDING";
  } else if(d.stale){
    state = "Not recording (recorder stopped unexpectedly)";
    note = "The last heartbeat went stale — the recording service may have died.";
    noteWarn = true;
  } else {
    state = "Idle";
  }
  if(!note && d.last_command && d.last_command.message){
    note = d.last_command.state+": "+d.last_command.message;
    noteWarn = d.last_command.state==="failed";
  }
  document.getElementById("recstate").textContent = state;
  const noteEl=document.getElementById("recnote");
  noteEl.textContent = note;
  noteEl.className = "recnote"+(noteWarn?" warn":"");

  document.getElementById("recmeta").innerHTML =
    `<div><span class="k">Participant</span><span class="v">${
        d.participant?("<code>"+esc(d.participant)+"</code>"):"—"}</span></div>`+
    `<div><span class="k">Session</span><span class="v">${
        d.session?("<code>"+esc(d.session)+"</code>"):"—"}</span></div>`+
    `<div><span class="k">Elapsed</span><span class="v" id="recelapsed">—</span></div>`+
    `<div><span class="k">Bag / video</span><span class="v">${
        d.bag_gb.toFixed(2)} / ${d.video_gb.toFixed(2)} GB</span></div>`;

  // Buttons: only one action is ever valid, so disable the other rather than
  // letting the operator fire a stop at an idle rig.
  const start=document.getElementById("btn-start");
  const stop=document.getElementById("btn-stop");
  const busy = d.last_command && d.last_command.state==="running";
  start.disabled = !d.service_up || d.active || busy;
  stop.disabled  = !d.service_up || !d.active || busy;
}

function paintRecording(d){
  recState=d;
  const el=document.getElementById("recelapsed");
  if(el) el.textContent = d.session ? fmtDur(d.elapsed_s) : "—";
  // Stale heartbeats stop advancing; flag it rather than freeze silently.
  const noteEl=document.getElementById("recnote");
  if(el && d.stale && noteEl && !noteEl.textContent)
    noteEl.textContent="heartbeat stale";
}

async function pollRecording(){
  try{
    const d = await (await fetch("/api/recording")).json();
    renderRecording(d); paintRecording(d);
  }catch(e){ /* the health poll owns the disconnected banner */ }
}

async function refresh(){
  let d;
  try { d = await (await fetch("/api/health")).json(); }
  catch(e){ document.getElementById("clock").textContent="disconnected"; return; }
  const s=d.summary;
  document.getElementById("clock").textContent =
    "updated "+new Date(d.time*1000).toLocaleTimeString()+" · "+d.window_s+"s window";
  const disk=d.disk||{}, sess=d.session||{};
  document.getElementById("tiles").innerHTML =
    tile('<span class="dot s-ok"></span>Healthy', s.ok, "of "+s.total+" topics")+
    tile('<span class="dot s-degraded"></span>Degraded', s.degraded, "below expected Hz")+
    tile('<span class="dot s-silent"></span>Silent', s.silent, "no data")+
    tile('<span class="dot s-waiting"></span>Waiting', s.waiting, "not yet seen")+
    tile("Disk free", (disk.free_gb!=null?disk.free_gb+" GB":"—"),
         (disk.used_pct!=null?disk.used_pct+"% used":"")+" "+(disk.path||""))+
    (sess.name?tile("Latest session", sess.bag_gb+" GB",
         sess.name+" · "+sess.age_s+"s ago"):"");
  const main=document.getElementById("main");
  main.innerHTML = d.groups.map(g=>`
    <div class="group"><h2>${g.name}</h2>
    <table><thead><tr>
      <th>Topic</th><th style="text-align:right">Hz</th>
      <th style="text-align:right">Expected</th><th>Trend</th><th>Status</th>
    </tr></thead><tbody>
    ${g.topics.map(t=>`<tr>
      <td class="topic">${t.topic}<span class="badge">${t.sink}</span></td>
      <td class="num">${t.hz.toFixed(1)}</td>
      <td class="num muted">${t.expected_hz!=null?t.expected_hz:"—"}</td>
      <td><svg class="spark" width="${SPARK_W}" height="${SPARK_H}">
        <path d="${sparkPath(t.spark)}" fill="none"
          stroke="var(--${t.status==='ok'?'ok':t.status==='degraded'?'degraded':t.status==='silent'?'silent':'waiting'})"
          stroke-width="1.5"/></svg></td>
      <td><span class="pill pill-${t.status}">
        <span class="dot s-${t.status}"></span>${t.status}</span></td>
    </tr>`).join("")}
    </tbody></table></div>`).join("");
}
refresh(); pollRecording();
setInterval(refresh, 1000);
// The recording panel polls faster than the topic table: it is the control the
// operator is watching, and it costs one small file read server-side.
setInterval(pollRecording, 700);
// Tick the elapsed timer locally so it advances smoothly between polls.
setInterval(()=>{ if(recState) paintRecording(recState); }, 1000);
</script>
</body>
</html>"""


def _build_app(node: TopicHealthMonitor) -> FastAPI:
    app = FastAPI(title="Rig topic health")

    @app.get("/", response_class=HTMLResponse)
    def index():
        return INDEX_HTML

    @app.get("/api/health")
    def health():
        return JSONResponse(node.snapshot())

    @app.get("/api/recording")
    def recording():
        return JSONResponse(node.recording_info())

    @app.post("/api/recording/{action}")
    def recording_command(action: str):
        """Start or stop a recording without holding the request open.

        Both are answered immediately: the heavy work happens on a background
        thread and the dashboard polls ``/api/recording``. A stop in particular
        can take the manager's whole ``stop_grace_s`` (30 s) to finalise the
        bag, which would otherwise look like a hung browser.
        """
        if action not in ("start", "stop"):
            return JSONResponse(
                {"ok": False, "message": f"unknown action {action!r}"}, status_code=400
            )
        state = node.recording_info()
        if not state["service_up"]:
            return JSONResponse(
                {"ok": False, "message": "recording service is not running"},
                status_code=409,
            )
        if action == "start" and state["active"]:
            return JSONResponse(
                {"ok": False, "message": "already recording"}, status_code=409
            )
        if action == "stop" and not state["active"]:
            return JSONResponse(
                {"ok": False, "message": "not recording"}, status_code=409
            )
        return JSONResponse(node.request_recording(action), status_code=202)

    return app


def main(args=None):
    rclpy.init(args=args)
    node = TopicHealthMonitor()

    config = uvicorn.Config(
        _build_app(node), host=node.host, port=node.port, log_level="warning"
    )
    server = uvicorn.Server(config)
    web_thread = threading.Thread(target=server.run, daemon=True)
    web_thread.start()
    node.get_logger().info(f"Topic health UI on http://{node.host}:{node.port}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.should_exit = True
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
