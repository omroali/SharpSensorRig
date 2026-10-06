#!/usr/bin/env python3
"""Recording-safety helpers that need no ROS runtime (pure, unit-tested).

This is the testable half of the 2026-10-01 hardening work. `recording_manager`
and `colour_video_recorder` own the ROS side (nodes, logging, subprocesses);
everything here is a decision or a small file format that can be exercised on a
workstation with no ROS, no sensors and no GPU:

* ``bag_split_args``      — rosbag2 ``--max-bag-size`` / ``--max-bag-duration``
                            so one crash cannot destroy a whole session (A1)
* ``StopRequest`` / ``install_stop_handlers``
                          — SIGHUP/SIGTERM become a cooperative stop flag that
                            the node turns into a graceful teardown (A2)
* ``poll_children`` / ``dead_children``
                          — watchdog verdicts over the recorder children (A4)
* ``video_startup_problems``
                          — missing/0-byte video detection (the 2026-10-01
                            failure mode) on the operator's console (B3)
* heartbeat / encoder status / stream failure files
                          — machine-readable state shared between the recorder
                            process and the manager, and readable from the host
                            after the container is gone (A6/B6/B8)

Nothing in this module imports rclpy, opens a session directory at import time,
or starts a thread at import time.
"""

from __future__ import annotations

import glob
import json
import os
import re
import signal
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Iterable, Mapping

#: Split the bag every N seconds: a crash then costs at most one split.
DEFAULT_BAG_MAX_DURATION_S = 300.0
#: Split the bag at this size as well, in case a sensor floods the topic set.
DEFAULT_BAG_MAX_SIZE_MB = 1024
#: Watchdog period. Children are polled this often while a recording is active.
DEFAULT_WATCHDOG_INTERVAL_S = 5.0
#: How long to let recorders drain before SIGKILL during a graceful stop.
DEFAULT_STOP_GRACE_S = 30.0
#: Wait this long after start before declaring a missing/0-byte video a failure
#: (the recorders hold the first ~2-5 s of frames back to measure the real fps).
DEFAULT_VIDEO_STARTUP_GRACE_S = 20.0
#: Consecutive watchdog ticks with no video growth before alarming.
VIDEO_STALL_TICKS = 3
#: The bag cache can legitimately hold writes for ~1 minute, so its stall alarm
#: is much more conservative (and a warning, not an error).
BAG_STALL_TICKS = 12

WATCHDOG_LOG_NAME = "watchdog.log"
HEARTBEAT_NAME = "recorder_heartbeat.json"
ENCODER_STATUS_NAME = "encoder_status.json"
STREAM_FAILURES_NAME = "stream_failures.json"
VIDEOS_DIR_NAME = "videos"


# --------------------------------------------------------------------------- #
# Bag splitting
# --------------------------------------------------------------------------- #
def bag_split_args(
    max_size_mb: float | int | None, max_duration_s: float | int | None
) -> list[str]:
    """rosbag2 splitting arguments; 0/None means "no limit" (old behaviour).

    With splitting, each completed ``bag_N.mcap`` is finalised with its own
    footer/ChunkIndex, so a crash loses at most the split in progress instead of
    the whole session.
    """
    args: list[str] = []
    if max_size_mb is not None and float(max_size_mb) > 0:
        args += ["--max-bag-size", str(int(float(max_size_mb) * 1024 * 1024))]
    if max_duration_s is not None and float(max_duration_s) > 0:
        args += ["--max-bag-duration", str(int(float(max_duration_s)))]
    return args


# --------------------------------------------------------------------------- #
# Session file layout (shared naming, mirrors session_check/topic_to_filename)
# --------------------------------------------------------------------------- #
def topic_to_filename(topic: str) -> str:
    """Mirror of ``colour_video_recorder.topic_to_filename`` / session_check."""
    return str(topic).strip("/").replace("/", "_")


def videos_dir(session_dir: str) -> str:
    return os.path.join(session_dir, VIDEOS_DIR_NAME)


def video_path(session_dir: str, topic: str) -> str:
    return os.path.join(videos_dir(session_dir), f"{topic_to_filename(topic)}.mp4")


def encoder_status_path(session_dir: str) -> str:
    return os.path.join(videos_dir(session_dir), ENCODER_STATUS_NAME)


def stream_failures_path(session_dir: str) -> str:
    return os.path.join(videos_dir(session_dir), STREAM_FAILURES_NAME)


def heartbeat_path(session_dir: str) -> str:
    return os.path.join(session_dir, HEARTBEAT_NAME)


def watchdog_log_path(session_dir: str) -> str:
    return os.path.join(session_dir, WATCHDOG_LOG_NAME)


def video_startup_problems(
    session_dir: str, topics: Iterable[str], min_bytes: int = 1
) -> list[str]:
    """Return one message per expected video that is missing or empty.

    This is the live equivalent of ``session_check._check_video``'s 0-byte test:
    on 2026-10-01 all four streams failed within 4 s of the start and nobody was
    told. Called by the watchdog after the startup grace period.
    """
    problems: list[str] = []
    for topic in topics:
        path = video_path(session_dir, topic)
        try:
            size = os.path.getsize(path)
        except OSError:
            problems.append(f"{topic}: no video file yet ({path})")
            continue
        if size < int(min_bytes):
            problems.append(f"{topic}: video is {size} bytes ({path})")
    return problems


def total_size(paths: Iterable[str]) -> int:
    total = 0
    for path in paths:
        try:
            total += os.path.getsize(path)
        except OSError:
            continue
    return total


def video_file_sizes(session_dir: str, topics: Iterable[str]) -> dict[str, int]:
    sizes: dict[str, int] = {}
    for topic in topics:
        path = video_path(session_dir, topic)
        try:
            sizes[str(topic)] = os.path.getsize(path)
        except OSError:
            sizes[str(topic)] = -1  # -1: not created yet
    return sizes


def bag_file_sizes(session_dir: str) -> dict[str, int]:
    """Sizes of every ``bag*/bag_N.*`` file (mcap, sqlite3 or wrapped).

    Storage-agnostic on purpose: the watchdog's stall check must work whichever
    ``bag_storage``/``bag_compression`` the config selects.
    """
    sizes: dict[str, int] = {}
    for path in sorted(glob.glob(os.path.join(session_dir, "bag*", "bag_*"))):
        if not os.path.isfile(path):
            continue
        try:
            sizes[os.path.relpath(path, session_dir)] = os.path.getsize(path)
        except OSError:
            continue
    return sizes


# --------------------------------------------------------------------------- #
# Atomic JSON / append-only event log
# --------------------------------------------------------------------------- #
def write_atomic_json(path: str, payload: Mapping[str, Any]) -> str:
    """Write JSON via ``tmp`` + ``os.replace`` so a reader never sees a half file."""
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    tmp_path = f"{path}.tmp.{os.getpid()}"
    with open(tmp_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True, default=str)
        handle.write("\n")
    os.replace(tmp_path, path)
    return path


def read_json(path: str) -> dict:
    """Read a JSON object; a missing/corrupt file reads as ``{}`` (never raises)."""
    try:
        with open(path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
    except (OSError, ValueError):
        return {}
    return payload if isinstance(payload, dict) else {}


def append_event_log(path: str, message: str, *, now: datetime | None = None) -> str:
    """Append one timestamped line to a session event log; returns the line."""
    stamp = (now or datetime.now()).isoformat(timespec="seconds")
    line = f"{stamp} {message}\n"
    try:
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, "a", encoding="utf-8") as handle:
            handle.write(line)
    except OSError:
        pass
    return line


# --------------------------------------------------------------------------- #
# Encoder / stream status files (recorder writes, manager reads)
# --------------------------------------------------------------------------- #
def write_encoder_status(session_dir: str, payload: Mapping[str, Any]) -> str:
    return write_atomic_json(encoder_status_path(session_dir), payload)


def read_encoder_status(session_dir: str) -> dict:
    return read_json(encoder_status_path(session_dir))


def write_stream_failures(session_dir: str, failures: Mapping[str, Any]) -> str:
    return write_atomic_json(
        stream_failures_path(session_dir),
        {"updated_at": datetime.now().isoformat(timespec="seconds"), "failures": dict(failures)},
    )


def read_stream_failures(session_dir: str) -> dict:
    payload = read_json(stream_failures_path(session_dir))
    failures = payload.get("failures")
    return failures if isinstance(failures, dict) else {}


def new_failure_keys(seen: Iterable[str], failures: Mapping[str, Any]) -> list[str]:
    """Failure keys present now but not in ``seen`` (for one-shot console alarms)."""
    known = set(seen)
    return [str(key) for key in failures if str(key) not in known]


# --------------------------------------------------------------------------- #
# Heartbeat
# --------------------------------------------------------------------------- #
def build_heartbeat(
    *,
    session_dir: str,
    participant: str,
    active: bool,
    started_at: float | None,
    children: Mapping[str, "ChildStatus"],
    bag_restarts: int,
    watchdog_events: Iterable[str] = (),
    video_sizes: Mapping[str, int] | None = None,
    bag_sizes: Mapping[str, int] | None = None,
    now: float | None = None,
) -> dict:
    """Small, stable JSON document: the host-side watchdog's only input (B7)."""
    now = time.time() if now is None else now
    return {
        "time": now,
        "iso": datetime.fromtimestamp(now).isoformat(timespec="seconds"),
        "pid": os.getpid(),
        "participant": participant,
        "session_dir": session_dir,
        "active": bool(active),
        "started_at": started_at,
        "elapsed_s": round(now - started_at, 1) if started_at else None,
        "children": {
            name: {"pid": status.pid, "alive": status.alive, "returncode": status.returncode}
            for name, status in children.items()
        },
        "bag_restarts": int(bag_restarts),
        "watchdog_events": list(watchdog_events),
        "video_bytes": dict(video_sizes or {}),
        "bag_bytes": dict(bag_sizes or {}),
    }


# --------------------------------------------------------------------------- #
# Child-process watchdog
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class ChildStatus:
    name: str
    pid: int | None
    alive: bool
    returncode: int | None = None

    def summary(self) -> str:
        if self.alive:
            return f"{self.name} (pid {self.pid}) alive"
        return f"{self.name} (pid {self.pid}) exited with code {self.returncode}"


def poll_children(children: Mapping[str, Any]) -> list[ChildStatus]:
    """Poll every ``Popen``-like child; ``None`` entries are skipped."""
    statuses: list[ChildStatus] = []
    for name, proc in children.items():
        if proc is None:
            continue
        pid = getattr(proc, "pid", None)
        try:
            returncode = proc.poll()
        except Exception:  # pragma: no cover - a wedged Popen reads as dead
            statuses.append(ChildStatus(str(name), pid, False, None))
            continue
        statuses.append(
            ChildStatus(str(name), pid, returncode is None, returncode)
        )
    return statuses


def dead_children(children: Mapping[str, Any]) -> list[ChildStatus]:
    return [status for status in poll_children(children) if not status.alive]


# --------------------------------------------------------------------------- #
# Signal handling
# --------------------------------------------------------------------------- #
def signal_name(signum: int | None) -> str:
    if signum is None:
        return "unknown"
    try:
        return signal.Signals(int(signum)).name
    except (ValueError, TypeError):
        return f"signal {signum}"


@dataclass
class StopRequest:
    """Cooperative stop flag set from a signal handler.

    The handler itself must stay trivial (it runs between bytecodes of whatever
    the main thread was doing, possibly while it holds a lock), so it only sets
    this flag. The node's main loop notices it and does the logging, the MCAP
    finalisation and the session report on the main thread.
    """

    event: threading.Event = field(default_factory=threading.Event, repr=False)
    signal_number: int | None = None
    received_at: float | None = None

    def request(self, signum: int | None = None) -> None:
        if signum is not None and self.signal_number is None:
            self.signal_number = int(signum)
            self.received_at = time.time()
        self.event.set()

    @property
    def requested(self) -> bool:
        return self.event.is_set()

    def wait(self, timeout: float | None = None) -> bool:
        return self.event.wait(timeout)

    def clear(self) -> None:
        self.signal_number = None
        self.received_at = None
        self.event.clear()

    def reason(self) -> str:
        if self.signal_number is None:
            return "stop requested"
        return f"{signal_name(self.signal_number)} received"


def install_stop_handlers(
    request: StopRequest,
    signals: Iterable[int] = (signal.SIGHUP, signal.SIGTERM),
) -> list[tuple[int, Any]]:
    """Route ``signals`` to ``request``; returns ``[(signum, previous), ...]``.

    SIGHUP is what a dying terminal/tmux delivers; SIGTERM is what a supervisor
    or ``docker stop`` delivers. With the flag set, the node's main loop leaves
    ``spin`` and runs the same teardown as ``/stop_recording`` — which is the
    only path that writes the MCAP footer, ``metadata.yaml`` and
    ``session_report.yaml``.
    """
    installed: list[tuple[int, Any]] = []

    def _handler(signum, _frame):
        request.request(signum)

    for signum in signals:
        try:
            installed.append((int(signum), signal.signal(int(signum), _handler)))
        except (ValueError, OSError, AttributeError, RuntimeError):
            # Not the main thread, or the platform has no such signal: the
            # other handlers still work, so degrade rather than fail.
            continue
    return installed


def restore_handlers(installed: Iterable[tuple[int, Any]]) -> None:
    for signum, previous in installed:
        try:
            signal.signal(int(signum), previous)
        except (ValueError, OSError, AttributeError, RuntimeError, TypeError):
            continue


def ros_shutdown_exceptions() -> tuple:
    """Exception types meaning "the ROS context went away" (best effort).

    ``rclpy.spin_once`` raises one of these when the context is shut down from
    outside (e.g. rclpy's own SIGTERM handler); the caller must still run its
    ``finally`` teardown rather than dump a traceback. Returns an empty tuple
    when rclpy is unavailable, which makes ``except ros_shutdown_exceptions():``
    catch nothing.
    """
    found = []
    for name in ("ExternalShutdownException", "ShutdownException"):
        try:
            module = __import__("rclpy.executors", fromlist=[name])
            found.append(getattr(module, name))
        except (ImportError, AttributeError):
            continue
    return tuple(found)


# --------------------------------------------------------------------------- #
# Bag directory ordering (bag/, bag_1/, ... bag_10/)
# --------------------------------------------------------------------------- #
def bag_sort_key(path: str) -> str:
    """Zero-pad digit runs so ``bag_10`` sorts after ``bag_2``."""
    name = os.path.basename(os.path.dirname(str(path))) or str(path)
    return "".join(
        f"{int(part):010d}" if part.isdigit() else part
        for part in re.split(r"(\d+)", name)
    )


def bag_metadata_files(session_dir: str) -> list[str]:
    """Every ``bag*/metadata.yaml``, oldest bag first (numerically sorted)."""
    return sorted(
        glob.glob(os.path.join(session_dir, "bag*", "metadata.yaml")),
        key=bag_sort_key,
    )


# --------------------------------------------------------------------------- #
# Session-check output parsing
# --------------------------------------------------------------------------- #
#: Lines emitted by the `ros2 run` launcher rather than by the check itself.
#: `ros2 run` appends one after the tool's own output, so taking the last stdout
#: line blindly reports "[ros2run]: Process exited with failure 1" instead of the
#: check's summary -- which is the one line an operator actually reads.
_LAUNCHER_NOISE_PREFIXES = ("[ros2run]", "[ros2]", "[ERROR] [launch]")


def session_check_summary(stdout: str) -> str:
    """The check's own summary line, ignoring `ros2 run` launcher noise.

    ``session_check`` prints per-topic failures followed by a summary such as
    ``FAIL: 18/97 checks ok, 0 warnings (/path/session_report.yaml)``. When it is
    invoked through ``ros2 run``, the launcher appends
    ``[ros2run]: Process exited with failure 1`` after that, which is what a
    naive "last line" parser returns. Prefer the last line that came from the
    check itself.

    Falls back to the last non-empty line if everything looks like noise, so a
    future change to the launcher's output cannot leave the summary blank.
    """
    lines = [ln.strip() for ln in (stdout or "").splitlines() if ln.strip()]
    if not lines:
        return ""
    meaningful = [
        ln for ln in lines if not ln.startswith(_LAUNCHER_NOISE_PREFIXES)
    ]
    return (meaningful or lines)[-1]
