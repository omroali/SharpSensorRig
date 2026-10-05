"""Tests for session_check over a *split* (multi-bag) session.

The 2026-10-01 remediation splits the bag so each completed split is finalised
separately; a watchdog restart adds further ``bag_N/`` directories. The health
check must aggregate all of them, otherwise a split session is reported against
one split's worth of data (and fails everything).

No ROS runtime required. Run with:
  python3 -m pytest session_recorder/test/test_session_check.py
"""

import os
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from session_recorder import session_check  # noqa: E402


def _metadata(duration_s, counts, paths=None):
    return {
        "rosbag2_bagfile_information": {
            "duration": {"nanoseconds": int(duration_s * 1e9)},
            "relative_file_paths": paths or ["bag_0.mcap"],
            "topics_with_message_count": [
                {"topic_metadata": {"name": topic}, "message_count": count}
                for topic, count in counts.items()
            ],
        }
    }


def _session(tmp_path, expected, bags=None, videos=None):
    session = tmp_path / "session_20261006_090000"
    (session / "videos").mkdir(parents=True, exist_ok=True)
    manifest = {
        "session": {"participant": "test", "min_rate_factor": 0.5},
        "expected_topics": expected,
        "expected_regex": [],
        "bag_exclude_regex": "",
    }
    (session / "expected_topics.yaml").write_text(yaml.safe_dump(manifest))
    for name, metadata in (bags or {}).items():
        directory = session / name
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "metadata.yaml").write_text(yaml.safe_dump(metadata))
    for name, payload in (videos or {}).items():
        (session / "videos" / name).write_bytes(payload)
    return session


def _report(session):
    return yaml.safe_load((session / "session_report.yaml").read_text())


def _check(report, topic):
    return next(c for c in report["checks"] if c["topic"] == topic)


BAG_EXPECTED = [{"topic": "/tf", "expected_hz": 10.0, "sink": "bag"}]


def test_bag_counts_and_duration_are_summed_across_bag_dirs(tmp_path):
    """A restart's bag_1/ must count towards the session, not replace bag/."""
    session = _session(
        tmp_path,
        BAG_EXPECTED,
        bags={
            "bag": _metadata(10.0, {"/tf": 100}, ["bag_0.mcap", "bag_1.mcap"]),
            "bag_1": _metadata(10.0, {"/tf": 110}, ["bag_0.mcap"]),
        },
    )
    assert session_check.run_check(str(session)) == 0
    report = _report(session)
    assert report["status"] == "PASS"
    assert report["bag_count"] == 2
    assert report["bag_duration_s"] == 20.0
    assert len(report["bag_metadata_files"]) == 2
    tf = _check(report, "/tf")
    assert tf["count"] == 210  # 100 + 110, not just the newest bag
    assert tf["hz"] == 10.5  # 210 messages over 20 s
    assert tf["status"] == "ok"


def test_a_split_run_with_one_metadata_yaml_still_checks(tmp_path):
    """Splitting keeps one metadata.yaml listing every bag_N.mcap."""
    session = _session(
        tmp_path,
        BAG_EXPECTED,
        bags={
            "bag": _metadata(
                600.0,
                {"/tf": 6000},
                [f"bag_{i}.mcap" for i in range(2)],  # two 5-minute splits
            )
        },
    )
    assert session_check.run_check(str(session)) == 0
    report = _report(session)
    assert report["bag_count"] == 1
    assert _check(report, "/tf")["count"] == 6000


def test_a_bag_without_metadata_is_reported_when_it_is_the_only_one(tmp_path):
    """An unfinalised first bag must not hide a finalised restarted bag."""
    session = _session(
        tmp_path,
        BAG_EXPECTED,
        bags={"bag_1": _metadata(30.0, {"/tf": 300})},
    )
    (session / "bag").mkdir()  # crashed before writing metadata.yaml
    (session / "bag" / "bag_0.mcap").write_bytes(b"unfinalised")
    assert session_check.run_check(str(session)) == 0
    report = _report(session)
    assert report["bag_count"] == 1
    assert report["bag_metadata_files"] == [str(session / "bag_1" / "metadata.yaml")]


def test_no_metadata_at_all_fails_the_check(tmp_path):
    session = _session(tmp_path, BAG_EXPECTED)
    assert session_check.run_check(str(session)) == 1
    report = _report(session)
    assert report["status"] == "FAIL"
    assert report["bag_count"] == 0
    assert any("no bag metadata.yaml" in c["detail"] for c in report["checks"])


def test_zero_byte_video_is_a_hard_failure(tmp_path):
    """The exact 2026-10-01 signature: four 0-byte mp4 files."""
    session = _session(
        tmp_path,
        [{"topic": "/cam/color/image_raw", "expected_hz": 15.0, "sink": "video"}],
        videos={"cam_color_image_raw.mp4": b""},
    )
    assert session_check.run_check(str(session)) == 1
    video = _check(_report(session), "/cam/color/image_raw")
    assert video["status"] == "fail"
    assert "missing or empty video" in video["detail"]


def test_numeric_bag_order_is_used_for_the_last_bag_field(tmp_path):
    session = _session(
        tmp_path,
        BAG_EXPECTED,
        bags={
            "bag": _metadata(10.0, {"/tf": 100}),
            "bag_2": _metadata(10.0, {"/tf": 100}),
            "bag_10": _metadata(10.0, {"/tf": 100}),
        },
    )
    assert session_check.run_check(str(session)) == 0
    report = _report(session)
    assert report["bag_metadata"].endswith(os.path.join("bag_10", "metadata.yaml"))
    assert report["bag_count"] == 3
    assert _check(report, "/tf")["count"] == 300
