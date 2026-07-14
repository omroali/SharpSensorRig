"""Unit tests for topic derivation (session_recorder.topics).

Pure-function tests: no ROS runtime required. Run with:
  python3 -m pytest session_recorder/test/test_topics.py
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from session_recorder import topics  # noqa: E402


def _cams(*names, **extra):
    return {
        "cameras": {
            n: {"serial": f"serial_{n}", "record": True, **extra} for n in names
        }
    }


# --- Kinect: raw-vs-rectified defaults and topic names ---------------------

def test_kinect_defaults_record_raw_sd_depth_and_ir():
    setup = topics.kinect_setup(topics.recorded_cameras(_cams("kinect2_1")), None)
    assert "/kinect2_1/sd/image_depth/compressed" in setup["bag_topics"]
    # IR default 5 Hz -> throttled name lands in the bag, source in throttles.
    assert "/kinect2_1/sd/image_ir/compressed/throttled" in setup["bag_topics"]
    assert any(
        s.startswith("/kinect2_1/sd/image_ir/compressed:") for s in setup["bag_throttles"]
    )
    # Colour stays rectified (baked into H.265).
    assert "/kinect2_1/qhd/image_color_rect" in setup["color_topics"]


def test_kinect_records_both_qhd_and_sd_camera_info():
    setup = topics.kinect_setup(topics.recorded_cameras(_cams("kinect2_1")), None)
    assert "/kinect2_1/qhd/camera_info" in setup["bag_topics"]
    assert "/kinect2_1/sd/camera_info" in setup["bag_topics"]


def test_kinect_unrectified_qhd_depth_is_rejected():
    cfg = {"streams": {"depth": {"resolution": "qhd", "rectified": False}}}
    with pytest.raises(ValueError, match="unrectified depth only exists at sd"):
        topics.kinect_setup(topics.recorded_cameras(_cams("kinect2_1")), cfg)


def test_kinect_depth_at_native_fps_is_not_throttled():
    cfg = {"streams": {"depth": {"fps": 30}}}
    setup = topics.kinect_setup(topics.recorded_cameras(_cams("kinect2_1")), cfg)
    assert "/kinect2_1/sd/image_depth/compressed" in setup["bag_topics"]
    assert not any("image_depth" in s for s in setup["bag_throttles"])


def test_kinect_calibration_snapshot_dirs_use_serial():
    setup = topics.kinect_setup(topics.recorded_cameras(_cams("kinect2_1")), None)
    assert setup["calib_source_dirs"]
    assert setup["calib_source_dirs"][0].endswith("serial_kinect2_1")


def test_kinect_snapshot_can_be_disabled():
    setup = topics.kinect_setup(
        topics.recorded_cameras(_cams("kinect2_1")),
        {"snapshot_calibration": False},
    )
    assert setup["calib_source_dirs"] == []


# --- RealSense: depth compression + throttle -------------------------------

def test_realsense_usb_depth_compressed_and_throttled():
    cams = topics.recorded_cameras(
        {"cameras": {"rs1": {"serial": "111", "driver": "usb", "record": True}}}
    )
    setup = topics.realsense_setup(cams, {"depth": {"mode": "compressed", "fps": 15}})
    assert any(
        "depth/image_rect_raw/compressedDepth/throttled" in t
        for t in setup["bag_topics"]
    )
    assert any(":15" in s for s in setup["bag_throttles"])


def test_realsense_dds_compressed_depth_rejected():
    cams = topics.recorded_cameras(
        {"cameras": {"rs1": {"serial": "111", "driver": "dds", "record": True}}}
    )
    with pytest.raises(ValueError, match="no.*compressed depth|does not support"):
        topics.realsense_setup(cams, {"depth": {"mode": "compressed"}})


# --- Manifest encode/decode round-trip -------------------------------------

@pytest.mark.parametrize("hz", [None, 5, 30.0, 119.88])
def test_expected_entry_roundtrip(hz):
    entry = topics.expected_entry("/a/b", hz, "bag")
    topic, parsed_hz, sink = topics.parse_expected_entry(entry)
    assert topic == "/a/b"
    assert sink == "bag"
    if hz is None:
        assert parsed_hz is None
    else:
        assert parsed_hz == pytest.approx(float(hz))


def test_throttle_entry_format():
    out, spec = topics.throttle_entry("/cam/depth", 15)
    assert out == "/cam/depth/throttled"
    assert spec == "/cam/depth:/cam/depth/throttled:15"


# --- Full build: vicon exclude + dedupe + tf presence ----------------------

def test_build_includes_tf_and_vicon_exclude():
    setup = topics.build_recording_setup(
        _cams("kinect2_1"),
        {},
        {},
        {"vicon": {"enabled": True}},
    )
    assert "/tf" in setup["bag_topics"]
    assert "/tf_static" in setup["bag_topics"]
    assert setup["bag_regex"] == topics.DEFAULT_VICON_REGEX
    assert setup["bag_exclude_regex"] == topics.DEFAULT_VICON_EXCLUDE_REGEX
    assert setup["expected_regex"] == [f"{topics.DEFAULT_VICON_REGEX}|120"]


def test_build_dedupes_bag_topics():
    setup = topics.build_recording_setup(_cams("kinect2_1"), {}, {}, {})
    assert len(setup["bag_topics"]) == len(set(setup["bag_topics"]))


def test_every_bag_topic_has_a_manifest_entry():
    setup = topics.build_recording_setup(
        _cams("kinect2_1"),
        {"cameras": {"rs1": {"serial": "1", "driver": "usb", "record": True}}},
        {},
        {"realsense": {"depth": {"mode": "compressed", "fps": 15}}},
    )
    manifest_topics = {
        topics.parse_expected_entry(e)[0] for e in setup["expected_topics"]
    }
    for topic in setup["bag_topics"]:
        assert topic in manifest_topics, f"{topic} missing from manifest"
