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

import os
import re
import shutil
import threading
import time
from collections import deque

import rclpy
import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

from session_recorder.topics import (
    build_recording_setup,
    config_dir,
    load_yaml,
    parse_expected_entry,
)

DISCOVERY_PERIOD_S = 2.0
# History kept per topic for the sparkline (one point per UI poll).
SPARK_POINTS = 40


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

        self._load_targets()

        self._discovery_timer = self.create_timer(
            DISCOVERY_PERIOD_S, self._discover
        )
        self._discover()

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
</style>
</head>
<body>
<header>
  <h1>Sensor rig · topic health</h1>
  <span class="clock" id="clock">connecting…</span>
</header>
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
refresh(); setInterval(refresh, 1000);
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
