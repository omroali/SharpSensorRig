"""Unit tests for the scrubable player's pure helpers.

No ROS runtime required. Run with:
  python3 -m pytest session_recorder/test/test_session_player.py
"""

import os
import sys

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
