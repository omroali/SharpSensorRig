"""Tests for the health monitor's recording control surface.

The monitor is a ROS node, but everything that decides *what the dashboard
shows* is plain file and dict work, so these tests drive those methods on a
bare object rather than standing up a node. No ROS graph, no web server.

Run with:
  python3 -m pytest session_recorder/test/test_topic_health_monitor.py
"""

import json
import os
import sys
import time
import types

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

from _ros_stubs import ensure_ros_available  # noqa: E402

ensure_ros_available()

from session_recorder import hardening  # noqa: E402
from session_recorder import topic_health_monitor as thm  # noqa: E402


def _monitor(output_root, *, services=True, heartbeat=None, session="session_20261008_140442"):
    """A monitor-shaped object with only what recording_info() reads."""
    node = types.SimpleNamespace(
        _control_services={thm.START_SERVICE, thm.STOP_SERVICE} if services else set(),
        _output_root=str(output_root),
        _last_command={},
    )
    node._newest_session_dir = types.MethodType(
        thm.TopicHealthMonitor._newest_session_dir, node
    )
    node.recording_info = types.MethodType(thm.TopicHealthMonitor.recording_info, node)
    node.request_recording = types.MethodType(
        thm.TopicHealthMonitor.request_recording, node
    )
    # request_recording plays a cue on success; bind the real ones so a test
    # that does not care about audio still exercises the real code path (they
    # fail soft on an unreachable listener, which is production behaviour).
    node._cue_probe = types.MethodType(thm.TopicHealthMonitor._cue_probe, node)
    node._ensure_cue_server = types.MethodType(
        thm.TopicHealthMonitor._ensure_cue_server, node
    )
    node._cue = types.MethodType(thm.TopicHealthMonitor._cue, node)
    if heartbeat is not None:
        session_dir = os.path.join(str(output_root), "testing-updated", session)
        os.makedirs(session_dir, exist_ok=True)
        with open(hardening.heartbeat_path(session_dir), "w") as handle:
            json.dump(heartbeat, handle)
    return node


def _heartbeat(**overrides):
    payload = {
        "active": True,
        "participant": "testing-updated",
        "session_dir": "/home/ubuntu/data/testing-updated/session_20261008_140442",
        "started_at": time.time() - 42.0,
        "elapsed_s": 41.5,
        "video_bytes": {"/a": 1024 ** 3},
        "bag_bytes": {"bag/bag_0.mcap": 2 * 1024 ** 3},
        "children": {"rosbag_record": {"alive": True, "pid": 1, "returncode": None}},
    }
    payload.update(overrides)
    return payload


# --- service presence -------------------------------------------------------

def test_reports_service_down_when_services_absent(tmp_path):
    info = _monitor(tmp_path, services=False).recording_info()
    assert info["service_up"] is False
    assert info["active"] is False


def test_service_present_but_no_session_yet(tmp_path):
    """Service up, nothing ever recorded: not an error, just idle."""
    info = _monitor(tmp_path).recording_info()
    assert info["service_up"] is True
    assert info["active"] is False
    assert info["session"] == ""


# --- reading the manager's heartbeat ---------------------------------------

def test_active_recording_reports_participant_and_sizes(tmp_path):
    node = _monitor(tmp_path, heartbeat=_heartbeat())
    info = node.recording_info()
    assert info["active"] is True
    assert info["stale"] is False
    assert info["participant"] == "testing-updated"
    assert info["session"] == "session_20261008_140442"
    assert info["video_gb"] == 1.0
    assert info["bag_gb"] == 2.0
    assert info["children"]["rosbag_record"]["alive"] is True
    # elapsed is recomputed from started_at so the UI timer ticks every second
    assert info["elapsed_s"] >= 42.0


def test_stale_heartbeat_is_not_reported_as_recording(tmp_path):
    """The crashed-manager case: a glowing REC light over a dead recorder.

    The heartbeat's own ``active`` flag stays True after a clean stop too (it is
    built before the flag is cleared), so freshness of the *file* decides.
    """
    node = _monitor(tmp_path, heartbeat=_heartbeat(active=True))
    path = hardening.heartbeat_path(
        os.path.join(str(tmp_path), "testing-updated", "session_20261008_140442")
    )
    old = time.time() - (thm.HEARTBEAT_STALE_S + 30)
    os.utime(path, (old, old))

    info = node.recording_info()
    assert info["stale"] is True
    assert info["active"] is False, "a dead manager must never look like recording"


def test_stale_heartbeat_falls_back_to_the_recorded_elapsed(tmp_path):
    """With no live clock, show the manager's last known duration, not zero."""
    node = _monitor(tmp_path, heartbeat=_heartbeat(elapsed_s=123.4))
    path = hardening.heartbeat_path(
        os.path.join(str(tmp_path), "testing-updated", "session_20261008_140442")
    )
    old = time.time() - (thm.HEARTBEAT_STALE_S + 5)
    os.utime(path, (old, old))
    assert node.recording_info()["elapsed_s"] == 123.4


def test_explicitly_inactive_heartbeat_is_not_recording(tmp_path):
    node = _monitor(tmp_path, heartbeat=_heartbeat(active=False))
    assert node.recording_info()["active"] is False


def test_missing_session_directory_is_handled(tmp_path):
    """output_root exists but has no participant dirs at all."""
    info = _monitor(tmp_path).recording_info()
    assert info["active"] is False and info["participant"] == ""


def test_nonexistent_output_root_is_handled(tmp_path):
    info = _monitor(tmp_path / "does-not-exist").recording_info()
    assert info["active"] is False


def test_newest_session_wins(tmp_path):
    older = "session_20260101_000000"
    newer = "session_20261008_140442"
    _monitor(tmp_path, heartbeat=_heartbeat(), session=older)
    node = _monitor(tmp_path, heartbeat=_heartbeat(), session=newer)
    # make the older directory look older on disk
    old_dir = os.path.join(str(tmp_path), "testing-updated", older)
    old_time = time.time() - 86400
    os.utime(old_dir, (old_time, old_time))
    assert node.recording_info()["session"] == newer


def test_heartbeat_without_started_at_does_not_crash(tmp_path):
    node = _monitor(tmp_path, heartbeat=_heartbeat(started_at=None, elapsed_s=7.0))
    info = node.recording_info()
    assert info["elapsed_s"] == 7.0


# --- start/stop dispatch ----------------------------------------------------

def test_request_recording_is_non_blocking_and_records_outcome(tmp_path):
    """Start/stop must return immediately; the UI polls for the result.

    A stop can finalise a bag for the manager's whole stop_grace_s (30 s), so a
    synchronous call would look like a hung browser. The fake service call here
    blocks until released, proving the HTTP layer would have returned already.
    """
    import threading

    node = _monitor(tmp_path)
    calls = []
    release = threading.Event()
    entered = threading.Event()

    def slow_trigger(service):
        calls.append(service)
        entered.set()
        release.wait(timeout=5.0)
        return {"ok": True, "message": "Recording started in /data/x"}

    node._call_trigger = slow_trigger

    started = time.monotonic()
    state = node.request_recording("start")
    elapsed = time.monotonic() - started

    assert elapsed < 1.0, "request_recording blocked the caller"
    assert state["state"] == "running", "returns before the service call finishes"
    assert state["action"] == "start"

    assert entered.wait(timeout=5.0), "background thread never called the service"
    assert node._last_command["state"] == "running", "still in flight"

    release.set()
    for _ in range(500):
        if node._last_command.get("state") != "running":
            break
        time.sleep(0.01)
    assert node._last_command["state"] == "ok"
    assert calls == [thm.START_SERVICE]
    assert "Recording started" in node._last_command["message"]


def test_request_recording_stop_targets_the_stop_service(tmp_path):
    node = _monitor(tmp_path)
    seen = []
    node._call_trigger = lambda service: (seen.append(service), {"ok": True, "message": ""})[1]
    node.request_recording("stop")
    for _ in range(200):
        if node._last_command.get("state") != "running":
            break
        time.sleep(0.01)
    assert seen == [thm.STOP_SERVICE]


def test_failed_command_is_surfaced_not_raised(tmp_path):
    node = _monitor(tmp_path)
    node._call_trigger = lambda service: {"ok": False, "message": "not available"}
    node.request_recording("start")
    for _ in range(200):
        if node._last_command.get("state") != "running":
            break
        time.sleep(0.01)
    assert node._last_command["state"] == "failed"
    assert node._last_command["message"] == "not available"


# --- HTTP layer (route registration + guard rails) --------------------------

def _app_node(tmp_path, *, services=True, heartbeat=None, active=True):
    node = _monitor(tmp_path, services=services, heartbeat=heartbeat)
    node.request_recording = lambda action: {"action": action, "state": "running"}
    return node


def test_app_exposes_the_recording_endpoints(tmp_path):
    app = thm._build_app(_app_node(tmp_path, heartbeat=_heartbeat()))
    for route in ("/api/health", "/api/recording", "/api/recording/{action}"):
        assert route in app.routes, f"missing route {route}"


def test_recording_endpoint_reports_state(tmp_path):
    app = thm._build_app(_app_node(tmp_path, heartbeat=_heartbeat()))
    response = app.routes["/api/recording"]()
    assert response.status_code == 200
    assert response.body["participant"] == "testing-updated"
    assert response.body["active"] is True


def test_start_is_refused_when_already_recording(tmp_path):
    app = thm._build_app(_app_node(tmp_path, heartbeat=_heartbeat(active=True)))
    response = app.routes["/api/recording/{action}"]("start")
    assert response.status_code == 409
    assert "already recording" in response.body["message"]


def test_stop_is_refused_when_not_recording(tmp_path):
    app = thm._build_app(_app_node(tmp_path, heartbeat=_heartbeat(active=False)))
    response = app.routes["/api/recording/{action}"]("stop")
    assert response.status_code == 409
    assert "not recording" in response.body["message"]


def test_both_actions_refused_when_service_is_down(tmp_path):
    app = thm._build_app(_app_node(tmp_path, services=False, heartbeat=_heartbeat()))
    for action in ("start", "stop"):
        response = app.routes["/api/recording/{action}"](action)
        assert response.status_code == 409
        assert "not running" in response.body["message"]


def test_unknown_action_is_rejected(tmp_path):
    app = thm._build_app(_app_node(tmp_path, heartbeat=_heartbeat()))
    response = app.routes["/api/recording/{action}"]("explode")
    assert response.status_code == 400


def test_valid_actions_are_accepted(tmp_path):
    """Idle + start, and recording + stop, must both be allowed through."""
    app = thm._build_app(_app_node(tmp_path, heartbeat=_heartbeat(active=False)))
    assert app.routes["/api/recording/{action}"]("start").status_code == 202
    app = thm._build_app(_app_node(tmp_path, heartbeat=_heartbeat(active=True)))
    assert app.routes["/api/recording/{action}"]("stop").status_code == 202


def test_index_serves_the_recording_controls(tmp_path):
    app = thm._build_app(_app_node(tmp_path))
    html = app.routes["/"]()   # FastAPI wraps the returned str in HTMLResponse
    assert "btn-start" in html and "btn-stop" in html
    assert "/api/recording" in html
    assert "recpulse" in html, "the pulsing REC animation is the point of the panel"
    # the start command shown to the operator must actually exist
    assert "unified_recording.launch.py" in html


# --- performance: this panel must not cost the rig anything ----------------

def test_recording_info_is_cheap_enough_to_poll(tmp_path):
    """Guard the polling cost.

    The dashboard hits this every 700 ms. It must stay a small file read plus a
    shallow stat scan -- if someone later swaps in a full recursive size walk
    over the data drive, this fails rather than quietly loading the rig.
    """
    node = _monitor(tmp_path, heartbeat=_heartbeat())
    # a few decoy participants so the scan has something to skip
    for name in ("aaa", "bbb", "ccc"):
        d = os.path.join(str(tmp_path), name, "session_20260101_000000")
        os.makedirs(d, exist_ok=True)
        with open(hardening.heartbeat_path(d), "w") as handle:
            json.dump(_heartbeat(active=False), handle)

    node.recording_info()  # warm
    iterations = 50
    started = time.perf_counter()
    for _ in range(iterations):
        node.recording_info()
    per_call_ms = (time.perf_counter() - started) / iterations * 1000
    assert per_call_ms < 5.0, f"recording_info took {per_call_ms:.2f} ms per call"


# --- service discovery shape (the "service not running" regression) --------

def test_refresh_services_parses_the_real_rclpy_pair_shape(tmp_path):
    """rclpy returns (name, [types]) pairs, not bare names.

    Building a set directly from those pairs raises
    ``TypeError: unhashable type: 'list'`` (the type list is unhashable). The
    broad ``except Exception`` in _refresh_services swallowed that, so the cache
    stayed permanently empty and the dashboard announced "Recording service not
    running" over a service that was running -- with Start/Stop never enabling.
    """
    node = types.SimpleNamespace(
        get_service_names_and_types=lambda: [
            ("/start_recording", ["std_srvs/srv/Trigger"]),
            ("/stop_recording", ["std_srvs/srv/Trigger"]),
            ("/rosout", ["rcl_interfaces/srv/GetLoggerLevels"]),
        ]
    )
    node._refresh_services = types.MethodType(
        thm.TopicHealthMonitor._refresh_services, node
    )
    node._refresh_services()

    assert thm.START_SERVICE in node._control_services
    assert thm.STOP_SERVICE in node._control_services
    assert "/rosout" in node._control_services
    # and the thing that was actually broken:
    assert isinstance(next(iter(node._control_services)), str)
    assert not any(isinstance(x, tuple) for x in node._control_services)


def test_recording_info_sees_a_running_service(tmp_path):
    """End to end: real graph shape in, service_up true out."""
    node = _monitor(tmp_path, services=False, heartbeat=_heartbeat())
    node.get_service_names_and_types = lambda: [
        ("/start_recording", ["std_srvs/srv/Trigger"]),
        ("/stop_recording", ["std_srvs/srv/Trigger"]),
    ]
    node._refresh_services = types.MethodType(
        thm.TopicHealthMonitor._refresh_services, node
    )
    node._refresh_services()

    info = node.recording_info()
    assert info["service_up"] is True, "a running service must be recognised"
    assert info["active"] is True
    assert info["participant"] == "testing-updated"


def test_refresh_services_survives_a_dead_graph(tmp_path):
    def boom():
        raise RuntimeError("graph gone")

    node = types.SimpleNamespace(get_service_names_and_types=boom)
    node._refresh_services = types.MethodType(
        thm.TopicHealthMonitor._refresh_services, node
    )
    node._refresh_services()          # must not raise
    assert node._control_services == set()


def test_refresh_services_tolerates_odd_entries(tmp_path):
    """A malformed entry must not abort the whole refresh."""
    node = types.SimpleNamespace(
        get_service_names_and_types=lambda: [
            "/bare_string_service",
            ("/good", ["std_srvs/srv/Trigger"]),
            (),
        ]
    )
    node._refresh_services = types.MethodType(
        thm.TopicHealthMonitor._refresh_services, node
    )
    node._refresh_services()
    assert "/bare_string_service" in node._control_services
    assert "/good" in node._control_services


# --- attribute shadowing guard ---------------------------------------------
#
# rclpy.node.Node keeps its entities in private attributes and its own code
# indexes them directly (``destroy_node`` does ``self._services[0]``). Assigning
# one of those names in a subclass silently replaces live rclpy state.
#
# This bit us for real: caching the ROS graph under ``self._services`` clobbered
# Node's list of service entities, and the node died inside rclpy.spin() with
# "'str' object has no attribute '_executor_event'", then again in
# destroy_node() with "'set' object is not subscriptable".

#: Attributes owned by rclpy.node.Node / its base. Never assign these.
RCLPY_RESERVED = {
    "_services", "_clients", "_subscriptions", "_publishers", "_timers",
    "_nodes", "_guards", "_waitables", "_parameters", "_parameter_overrides",
    "_context", "_handle", "_default_callback_group", "_callback_groups",
}


def test_monitor_does_not_shadow_rclpy_node_internals():
    """A subclass must not reassign rclpy's own private attributes.

    Checked against the source rather than an instance, because constructing a
    real Node needs a live rclpy context -- and the failure mode only appears
    once rclpy spins or destroys the node, far from the mistake.
    """
    import re

    source = open(thm.__file__).read()
    assigned = set(re.findall(r"self\.(_[a-z_]+)\s*[:=]", source))
    clashes = sorted(assigned & RCLPY_RESERVED)
    assert not clashes, (
        f"topic_health_monitor assigns rclpy-internal attribute(s) {clashes}; "
        "rclpy.spin()/destroy_node() index these directly and will crash. "
        "Use a distinct name (e.g. _control_services)."
    )


def test_recording_info_reads_the_control_cache(tmp_path):
    """The renamed cache must still be what recording_info consults."""
    node = _monitor(tmp_path, heartbeat=_heartbeat(), services=True)
    assert node.recording_info()["service_up"] is True
    node._control_services = set()          # simulate services gone
    assert node.recording_info()["service_up"] is False


# --- audible start/stop cues -------------------------------------------------
#
# The container has no audio device, so the monitor asks a host listener
# (SHARP/rig_cue_server.py) to beep over loopback. These tests pin the two
# properties that matter: the cue fires only on a real transition, and a missing
# cue server never breaks recording control.

def _cue_node(tmp_path, trigger, monkeypatch, url="http://127.0.0.1:8766",
              ensure=None):
    node = _monitor(tmp_path)
    node._call_trigger = trigger
    node._cue = types.MethodType(thm.TopicHealthMonitor._cue, node)
    monkeypatch.setattr(thm, "CUE_SERVER_URL", url)
    # The listener is now made reachable before every cue. Stub that step by
    # default so these tests cover the POST itself; the ensure behaviour has its
    # own tests below.
    node._ensure_cue_server = ensure or (lambda: (True, ""))
    return node


def _wait_idle(node, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if node._last_command.get("state") != "running":
            return
        time.sleep(0.01)


def test_start_cue_fires_once_on_success(tmp_path, monkeypatch):
    node = _cue_node(tmp_path, lambda s: {"ok": True, "message": "started"}, monkeypatch)
    seen = []
    # stub the transport, not the decision: record which cue was requested
    def fake_urlopen(request, timeout=None):
        seen.append(request.full_url)
        return _FakeResponse(b'{"ok": true, "message": ""}')
    monkeypatch.setattr(thm.urllib.request, "urlopen", fake_urlopen)

    node.request_recording("start")
    _wait_idle(node)
    assert node._last_command["state"] == "ok"
    assert len(seen) == 1, f"expected exactly one cue, got {seen}"
    assert seen[0].endswith("/cue/start")


def test_stop_cue_targets_the_stop_endpoint(tmp_path, monkeypatch):
    node = _cue_node(tmp_path, lambda s: {"ok": True, "message": "stopped"}, monkeypatch)
    seen = []
    monkeypatch.setattr(thm.urllib.request, "urlopen", lambda request, timeout=None: (
        seen.append(request.full_url), _FakeResponse(b'{"ok": true, "message": ""}'))[1])

    node.request_recording("stop")
    _wait_idle(node)
    assert len(seen) == 1 and seen[0].endswith("/cue/stop")


def test_no_cue_when_the_recording_did_not_start(tmp_path, monkeypatch):
    """A silent failure must stay silent -- no beep for a recording that failed."""
    node = _cue_node(tmp_path, lambda s: {"ok": False, "message": "boom"}, monkeypatch)
    calls = []
    monkeypatch.setattr(thm.urllib.request, "urlopen",
                        lambda request, timeout=None: calls.append(request.full_url))

    node.request_recording("start")
    _wait_idle(node)
    assert node._last_command["state"] == "failed"
    assert calls == [], "a failed start must not beep"


def test_unreachable_cue_server_does_not_break_control(tmp_path, monkeypatch):
    """No speaker, or listener not running: recording still works."""
    node = _cue_node(tmp_path, lambda s: {"ok": True, "message": "started"}, monkeypatch)

    node._ensure_cue_server = lambda: (False, "cue server unreachable (refused)")
    node.request_recording("start")
    _wait_idle(node)
    assert node._last_command["state"] == "ok", "cue failure must not fail the command"
    assert "cue not played" in node._last_command["message"]
    assert "started" in node._last_command["message"]


def test_cue_helper_is_never_raising(tmp_path, monkeypatch):
    node = _monitor(tmp_path)
    node._cue = types.MethodType(thm.TopicHealthMonitor._cue, node)

    def explode(request, timeout=None):
        raise RuntimeError("something unexpected")

    node._ensure_cue_server = lambda: (True, "")
    monkeypatch.setattr(thm.urllib.request, "urlopen", explode)
    ok, detail = node._cue("start")
    assert ok is False and "cue failed" in detail


def test_cue_is_disabled_when_url_is_empty(tmp_path, monkeypatch):
    monkeypatch.setattr(thm, "CUE_SERVER_URL", "")
    node = _monitor(tmp_path)
    node._cue = types.MethodType(thm.TopicHealthMonitor._cue, node)
    ok, detail = node._cue("start")
    assert ok is False and "disabled" in detail


class _FakeResponse:
    def __init__(self, body: bytes):
        self._body = body

    def read(self):
        return self._body

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


# --- self-healing cue listener ----------------------------------------------
#
# The real failure: the listener was not running, the monitor POSTed a cue, got
# "connection refused", and the operator saw silence with no explanation. The
# cue path now asks a host helper to start the listener first.

def test_ensure_prefers_the_host_helper(tmp_path, monkeypatch):
    node = _monitor(tmp_path)
    node._ensure_cue_server = types.MethodType(
        thm.TopicHealthMonitor._ensure_cue_server, node
    )
    monkeypatch.setattr(thm, "CUE_SERVER_URL", "http://127.0.0.1:8766")
    monkeypatch.setattr(thm.shutil, "which", lambda name: "/usr/local/bin/" + name)

    calls = []

    class Done:
        stdout = '{"ok": true, "state": "started", "detail": ""}'

    def fake_run(argv, **kwargs):
        calls.append(argv)
        return Done()

    monkeypatch.setattr(thm.subprocess, "run", fake_run)
    ok, detail = node._ensure_cue_server()
    assert ok is True and detail == "started"
    assert calls == [[thm.CUE_ENSURE_BIN]], "helper must be invoked by name"


def test_ensure_reports_helper_failure_clearly(tmp_path, monkeypatch):
    node = _monitor(tmp_path)
    node._ensure_cue_server = types.MethodType(
        thm.TopicHealthMonitor._ensure_cue_server, node
    )
    monkeypatch.setattr(thm, "CUE_SERVER_URL", "http://127.0.0.1:8766")
    monkeypatch.setattr(thm.shutil, "which", lambda name: "/usr/local/bin/" + name)

    class Done:
        stdout = '{"ok": false, "state": "unavailable", "detail": "not found on the host"}'

    monkeypatch.setattr(thm.subprocess, "run", lambda argv, **kw: Done())
    ok, detail = node._ensure_cue_server()
    assert ok is False and "not found on the host" in detail


def test_ensure_falls_back_to_probing_when_helper_absent(tmp_path, monkeypatch):
    """A hand-started listener must still work without the helper installed."""
    node = _monitor(tmp_path)
    node._ensure_cue_server = types.MethodType(
        thm.TopicHealthMonitor._ensure_cue_server, node
    )
    node._cue_probe = lambda: (True, "")
    monkeypatch.setattr(thm, "CUE_SERVER_URL", "http://127.0.0.1:8766")
    monkeypatch.setattr(thm.shutil, "which", lambda name: None)
    assert node._ensure_cue_server() == (True, "")


def test_ensure_survives_a_helper_that_times_out(tmp_path, monkeypatch):
    node = _monitor(tmp_path)
    node._ensure_cue_server = types.MethodType(
        thm.TopicHealthMonitor._ensure_cue_server, node
    )
    monkeypatch.setattr(thm, "CUE_SERVER_URL", "http://127.0.0.1:8766")
    monkeypatch.setattr(thm.shutil, "which", lambda name: "/usr/local/bin/" + name)

    def boom(argv, **kwargs):
        raise thm.subprocess.TimeoutExpired(argv, 10.0)

    monkeypatch.setattr(thm.subprocess, "run", boom)
    ok, detail = node._ensure_cue_server()
    assert ok is False and "timed out" in detail


def test_ensure_survives_garbage_helper_output(tmp_path, monkeypatch):
    node = _monitor(tmp_path)
    node._ensure_cue_server = types.MethodType(
        thm.TopicHealthMonitor._ensure_cue_server, node
    )
    monkeypatch.setattr(thm, "CUE_SERVER_URL", "http://127.0.0.1:8766")
    monkeypatch.setattr(thm.shutil, "which", lambda name: "/usr/local/bin/" + name)

    class Done:
        stdout = "this is not json"

    monkeypatch.setattr(thm.subprocess, "run", lambda argv, **kw: Done())
    ok, detail = node._ensure_cue_server()
    assert ok is False and "cue helper failed" in detail


def test_ensure_is_skipped_when_cues_disabled(tmp_path, monkeypatch):
    node = _monitor(tmp_path)
    node._ensure_cue_server = types.MethodType(
        thm.TopicHealthMonitor._ensure_cue_server, node
    )
    monkeypatch.setattr(thm, "CUE_SERVER_URL", "")
    assert node._ensure_cue_server() == (False, "cues disabled")
