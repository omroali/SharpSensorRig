"""Unit tests for the scrubable player's pure helpers.

No ROS runtime required. Run with:
  python3 -m pytest session_recorder/test/test_session_player.py
"""

import os
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from session_recorder import session_player as sp  # noqa: E402


# --- frame_index_at --------------------------------------------------------

def test_frame_index_at_picks_last_frame_at_or_before():
    stamps = [100, 200, 300, 400]
    assert sp.frame_index_at(stamps, 50) == -1
    assert sp.frame_index_at(stamps, 100) == 0
    assert sp.frame_index_at(stamps, 250) == 1
    assert sp.frame_index_at(stamps, 400) == 3
    assert sp.frame_index_at(stamps, 999) == 3


def test_frame_index_at_empty():
    assert sp.frame_index_at([], 123) == -1


# --- activity_jump_target --------------------------------------------------

def _acts(*starts):
    return [{"index": i, "start_ns": s, "end_ns": s + 10} for i, s in enumerate(starts)]


def test_jump_forward_goes_to_next_start_strictly_after():
    acts = _acts(100, 200, 300)
    assert sp.activity_jump_target(acts, 150, +1) == 200
    assert sp.activity_jump_target(acts, 200, +1) == 300  # not the current one
    assert sp.activity_jump_target(acts, 300, +1) is None


def test_jump_backward_prefers_current_activity_start_then_previous():
    acts = _acts(100, 200, 300)
    assert sp.activity_jump_target(acts, 250, -1) == 200  # inside #1 -> its start
    assert sp.activity_jump_target(acts, 200, -1) == 100
    assert sp.activity_jump_target(acts, 100, -1) == 0    # clamp to session start


def test_jump_with_no_activities():
    assert sp.activity_jump_target([], 500, +1) is None
    assert sp.activity_jump_target([], 500, -1) is None


# --- progress_fraction -----------------------------------------------------

def test_progress_fraction_clamps():
    assert sp.progress_fraction(100, 200, 100) == 0.0
    assert sp.progress_fraction(100, 200, 150) == 0.5
    assert sp.progress_fraction(100, 200, 500) == 1.0
    assert sp.progress_fraction(100, 200, 0) == 0.0
    assert sp.progress_fraction(100, 100, 100) == 0.0  # degenerate range


# --- human_status ----------------------------------------------------------

def test_human_status_shows_play_state_time_and_activity():
    st = {
        "playing": True, "current_ns": 62_000_000_000, "start_ns": 0,
        "end_ns": 120_000_000_000, "fraction": 0.5167,
        "action_id": "A07", "action_name": "carry_tote", "tier": "T1",
        "activity_index": 3,
    }
    line = sp.human_status(st)
    assert line.startswith("[PLAY ] 01:02/02:00")
    assert "A07 carry_tote (T1)" in line and "#3" in line


def test_human_status_paused_with_no_activity():
    st = {
        "playing": False, "current_ns": 0, "start_ns": 0, "end_ns": 0,
        "fraction": 0.0, "action_id": None, "action_name": None,
        "tier": None, "activity_index": None,
    }
    line = sp.human_status(st)
    assert line.startswith("[PAUSE]")
    assert "no activity" in line


# --- _build_time_range -----------------------------------------------------

def _range_for(tmp_path, starting_time, duration, stamps):
    """SessionPlayer with only the bag metadata + one video CSV populated."""
    videos_dir = tmp_path / "videos"
    videos_dir.mkdir()
    csv_path = videos_dir / "cam.csv"
    csv_path.write_text(
        "frame_idx,ros_timestamp_ns\n"
        + "".join(f"{i},{ts}\n" for i, ts in enumerate(stamps))
    )
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text(yaml.safe_dump({
        "rosbag2_bagfile_information": {
            "starting_time": starting_time,
            "duration": {"nanoseconds": duration},
        }
    }))
    player = sp.SessionPlayer.__new__(sp.SessionPlayer)
    player._build_time_range(str(bag), [{"csv": str(csv_path)}])
    return player


def test_bag_start_reads_v9_nanoseconds_since_epoch(tmp_path):
    """Jazzy metadata v9 renamed the field; missing the key used to mean 0."""
    player = _range_for(
        tmp_path,
        {"nanoseconds_since_epoch": 1_788_957_146_998_855_419},
        953_432_074_939,
        [1_788_957_147_431_141_846, 1_788_958_100_425_352_295],
    )
    assert player.start_ns == 1_788_957_146_998_855_419
    assert player.end_ns == 1_788_958_100_430_930_358


def test_bag_start_still_reads_legacy_nanoseconds_key(tmp_path):
    player = _range_for(tmp_path, {"nanoseconds": 5_000}, 1_000, [4_500, 6_500])
    assert player.start_ns == 4_500
    assert player.end_ns == 6_500


def test_missing_bag_start_does_not_collapse_range_to_epoch(tmp_path):
    """No stated start -> video stamps define the range, not a phantom 0."""
    first = 1_788_957_147_431_141_846
    player = _range_for(tmp_path, {}, 1_000, [first, first + 5_000])
    assert player.start_ns == first
    assert player.end_ns == first + 5_000


# --- colour image frame adoption ------------------------------------------

class _FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(text)


class _FakeNode:
    def __init__(self):
        self.logger = _FakeLogger()

    def get_logger(self):
        return self.logger


class _FrameMsg:
    def __init__(self, frame_id):
        class _Header:
            pass

        self.header = _Header()
        self.header.frame_id = frame_id


def _stream():
    return {"spec": {"publish_topic": "/cam/color/image_raw"}, "frame_id": ""}


def test_adopt_video_frame_takes_recorded_frame():
    node = _FakeNode()
    stream = _stream()

    sp._adopt_video_frame(node, stream, "/cam/color/camera_info",
                          _FrameMsg("kinect2_1_rgb_optical_frame"))

    assert stream["frame_id"] == "kinect2_1_rgb_optical_frame"
    assert "kinect2_1_rgb_optical_frame" in node.logger.lines[0]


def test_adopt_video_frame_ignores_empty_and_keeps_first():
    node = _FakeNode()
    stream = _stream()

    sp._adopt_video_frame(node, stream, "/cam/color/camera_info", _FrameMsg(""))
    assert stream["frame_id"] == ""

    sp._adopt_video_frame(node, stream, "/cam/color/camera_info", _FrameMsg("cam_link"))
    sp._adopt_video_frame(node, stream, "/cam/color/camera_info", _FrameMsg("other_link"))
    assert stream["frame_id"] == "cam_link"
