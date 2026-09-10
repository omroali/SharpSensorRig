"""Unit tests for activity parsing/labelling (session_recorder.activities).

Pure-function tests: no ROS runtime required. Run with:
  python3 -m pytest session_recorder/test/test_activities.py
"""

import csv
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from session_recorder import activities as act  # noqa: E402


def _event(etype, ros_ns, payload=None, trial_uuid=None, uuid=None):
    return {
        "event_uuid": uuid or f"u{ros_ns}-{etype}",
        "ros_ns": ros_ns,
        "type": etype,
        "trial_uuid": trial_uuid,
        "payload": payload or {},
    }


def _t1_start(ns, action_id="A07", name="carry_tote", position=0, rep=1):
    return _event(
        "action_start", ns,
        {"action_id": action_id, "action_name": name, "position": position, "rep": rep, "source": "researcher"},
    )


def _t1_end(ns, action_id="A07", name="carry_tote", position=0, rep=1):
    return _event(
        "action_end", ns,
        {"action_id": action_id, "action_name": name, "position": position, "rep": rep, "source": "researcher"},
    )


# --- parse_activities ------------------------------------------------------

def test_t1_rep_pairs_start_with_end():
    events = [
        _event("session_start", 100),
        _t1_start(200, "A01", "walk", 0, 1),
        _t1_end(500, "A01", "walk", 0, 1),
        _t1_start(500, "A02", "reach_high", 1, 1),
        _t1_end(900, "A02", "reach_high", 1, 1),
        _event("session_end", 1000),
    ]
    acts = act.parse_activities(events)
    assert [(a["action_id"], a["start_ns"], a["end_ns"]) for a in acts] == [
        ("A01", 200, 500),
        ("A02", 500, 900),
    ]
    assert all(a["tier"] == "T1" for a in acts)
    # Indices are renumbered sequentially for jumping.
    assert [a["index"] for a in acts] == [0, 1]


def test_action_skipped_closes_and_flags():
    events = [
        _t1_start(100, "A04", "lift_box"),
        _event("action_skipped", 400, {"action_id": "A04", "action_name": "lift_box", "position": 0, "rep": 1}),
    ]
    acts = act.parse_activities(events)
    assert len(acts) == 1
    assert acts[0]["skipped"] is True
    assert acts[0]["end_ns"] == 400


def test_t2_activities_carry_trial_uuid_and_step_index():
    events = [
        _event("trial_start", 10, {}, trial_uuid="T-1"),
        _event("action_start", 100, {"action_id": "A11", "action_name": "open_box", "step_index": 0}, trial_uuid="T-1"),
        _event("action_end", 250, {"action_id": "A11", "action_name": "open_box", "step_index": 0}, trial_uuid="T-1"),
        _event("action_start", 250, {"action_id": "A12", "action_name": "fold_cardboard", "step_index": 1}, trial_uuid="T-1"),
        _event("action_end", 600, {"action_id": "A12", "action_name": "fold_cardboard", "step_index": 1}, trial_uuid="T-1"),
        _event("trial_end", 700, {"status": "done"}, trial_uuid="T-1"),
    ]
    acts = act.parse_activities(events)
    assert [a["tier"] for a in acts] == ["T2", "T2"]
    assert acts[0]["trial_uuid"] == "T-1" and acts[0]["step_index"] == 0
    assert acts[1]["step_index"] == 1


def test_unclosed_activity_is_closed_at_last_event():
    acts = act.parse_activities([_t1_start(100, "A15", "stand_idle"), _event("sync_mark", 300)])
    assert len(acts) == 1 and acts[0]["end_ns"] == 300


def test_out_of_order_events_are_sorted():
    events = [_t1_end(500, "A01", "walk"), _t1_start(200, "A01", "walk")]
    acts = act.parse_activities(events)
    assert acts[0]["start_ns"] == 200 and acts[0]["end_ns"] == 500


def test_t3_annotation_is_a_point_activity():
    events = [_event("t3_annotation", 123, {"action_id": "A20", "action_name": "count_items"}, trial_uuid="T-9")]
    acts = act.parse_activities(events)
    assert len(acts) == 1
    assert acts[0]["tier"] == "T3" and acts[0]["start_ns"] == acts[0]["end_ns"] == 123


# --- ActivityIndex / labelling ---------------------------------------------

def test_activity_at_is_containment_not_nearest():
    acts = act.parse_activities([
        _t1_start(200, "A01", "walk"), _t1_end(500, "A01", "walk"),
        _t1_start(500, "A02", "reach_high"), _t1_end(900, "A02", "reach_high"),
    ])
    index = act.ActivityIndex(acts)
    assert index.activity_at(100) is None            # before the first
    assert index.activity_at(200)["action_id"] == "A01"
    assert index.activity_at(700)["action_id"] == "A02"
    assert index.activity_at(900)["action_id"] == "A02"  # end inclusive
    assert index.activity_at(901) is None            # after the last


def test_label_video_csv_and_write(tmp_path):
    acts = act.parse_activities([
        _t1_start(1_000, "A01", "walk", 0, 1), _t1_end(2_000, "A01", "walk", 0, 1),
    ])
    index = act.ActivityIndex(acts)
    csv_path = tmp_path / "stream.csv"
    csv_path.write_text(
        "frame_idx,ros_timestamp_ns\n0,500\n1,1500\n2,2500\n"
    )
    rows = act.label_video_csv(str(csv_path), index)
    assert [r["action_id"] for r in rows] == ["", "A01", ""]
    assert rows[1]["tier"] == "T1" and rows[1]["rep"] == 1

    out = tmp_path / "labels" / "stream.csv"
    act.write_labels(rows, str(out))
    with open(out, newline="") as handle:
        written = list(csv.DictReader(handle))
    assert written[1]["action_name"] == "walk"
    assert written[0]["activity_index"] == ""
