"""Unit tests for replay static-transform overrides.

Pure-function tests: no ROS runtime required. Run with:
  python3 -m pytest session_recorder/test/test_tf_overrides.py
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from session_recorder import tf_overrides as ov  # noqa: E402


class FakeTransform:
    """Stand-in for geometry_msgs/TransformStamped (only what plan needs)."""

    class _Header:
        def __init__(self, frame_id):
            self.frame_id = frame_id

    class _Transform:
        def __init__(self, q):
            self.rotation = q

    class _Quat:
        def __init__(self, x, y, z, w):
            self.x, self.y, self.z, self.w = x, y, z, w

    def __init__(self, parent, child, quat=(0.0, 0.0, 0.0, 1.0)):
        self.header = self._Header(parent)
        self.child_frame_id = child
        self.transform = self._Transform(self._Quat(*quat))


# ── parsing ─────────────────────────────────────────────────────────────────


def test_parse_spec_position_only():
    parsed = ov.parse_override_spec("realsense_d55_1_link=-3.454,-0.404,1.022")
    assert parsed["child"] == "realsense_d55_1_link"
    assert parsed["parent"] == "map"
    assert parsed["position"] == pytest.approx((-3.454, -0.404, 1.022))
    assert parsed["rotation"] is None


def test_parse_spec_with_parent_and_rotation():
    parsed = ov.parse_override_spec("cam=base:-1,2,3@0,0,-90")
    assert parsed["parent"] == "base"
    assert parsed["position"] == pytest.approx((-1.0, 2.0, 3.0))
    assert parsed["rotation"] == pytest.approx((0.0, 0.0, -90.0))


@pytest.mark.parametrize("spec", ["nonsense", "=1,2,3", "cam=1,2", "cam=1,2,3@1,2"])
def test_parse_spec_rejects_bad_input(spec):
    with pytest.raises(ValueError):
        ov.parse_override_spec(spec)


def test_load_overrides_reads_yaml(tmp_path):
    session = tmp_path / "session_x"
    session.mkdir()
    (session / "tf_overrides.yaml").write_text(
        "overrides:\n"
        "  - child: realsense_d55_1_link\n"
        "    position: {x: -3.454, y: -0.404, z: 1.022}\n"
        "  - parent: base\n"
        "    child: other_link\n"
        "    position: [1, 2, 3]\n"
        "    orientation: {roll_deg: 0, pitch_deg: 0, yaw_deg: -90}\n",
        encoding="utf-8",
    )
    overrides = ov.load_overrides(str(session))
    assert [o["child"] for o in overrides] == ["realsense_d55_1_link", "other_link"]
    assert overrides[0]["parent"] == "map"
    assert overrides[0]["rotation"] is None
    assert overrides[1]["parent"] == "base"
    assert overrides[1]["rotation"] == pytest.approx((0.0, 0.0, -90.0))


def test_load_overrides_missing_file_is_empty(tmp_path):
    assert ov.load_overrides(str(tmp_path)) == []


def test_merge_overrides_cli_wins_over_file():
    from_file = [ov.parse_override_spec("cam=1,1,1"), ov.parse_override_spec("other=2,2,2")]
    from_cli = [ov.parse_override_spec("cam=9,9,9")]

    merged = ov.merge_overrides(from_file, from_cli)

    assert [o["child"] for o in merged] == ["cam", "other"]
    assert merged[0]["position"] == pytest.approx((9.0, 9.0, 9.0))
    assert merged[1]["position"] == pytest.approx((2.0, 2.0, 2.0))


# ── maths ───────────────────────────────────────────────────────────────────


def test_quat_from_rpy_deg_identity_and_yaw90():
    assert ov.quat_from_rpy_deg((0.0, 0.0, 0.0)) == pytest.approx((0.0, 0.0, 0.0, 1.0))
    x, y, z, w = ov.quat_from_rpy_deg((0.0, 0.0, 90.0))
    assert (x, y, z) == pytest.approx((0.0, 0.0, math.sin(math.pi / 4)))
    assert w == pytest.approx(math.cos(math.pi / 4))


# ── planning ────────────────────────────────────────────────────────────────


def test_plan_drops_every_parent_of_an_overridden_child():
    """The duplicated map->realsense and kinect->realsense edges both go."""
    transforms = [
        FakeTransform("map", "realsense_d55_1_link", (0.5, 0.5, 0.5, 0.5)),
        FakeTransform("kinect2_1_link", "realsense_d55_1_link"),
        FakeTransform("map", "kinect2_1_link"),
        FakeTransform("map", "vicon"),
    ]
    overrides = [ov.normalize_override(
        {"child": "realsense_d55_1_link", "position": {"x": -3.454, "y": -0.404, "z": 1.022}}
    )]

    kept, applied = ov.plan_overrides(transforms, overrides)

    assert [t.child_frame_id for t in kept] == ["kinect2_1_link", "vicon"]
    assert len(applied) == 1
    assert applied[0]["parent"] == "map"
    assert applied[0]["replaced"] == 2
    assert applied[0]["position"] == pytest.approx((-3.454, -0.404, 1.022))
    # No explicit rotation -> reuse the recorded one from the matching parent.
    assert applied[0]["rotation_source"] == "recorded"
    assert applied[0]["quat"] == pytest.approx((0.5, 0.5, 0.5, 0.5))


def test_plan_falls_back_to_first_recorded_rotation():
    transforms = [FakeTransform("kinect2_1_link", "cam", (0.0, 0.0, 0.7071, 0.7071))]
    overrides = [ov.normalize_override({"child": "cam", "position": [0, 0, 1]})]

    _, applied = ov.plan_overrides(transforms, overrides)

    assert applied[0]["rotation_source"] == "recorded"
    assert applied[0]["quat"] == pytest.approx((0.0, 0.0, 0.7071, 0.7071))


def test_plan_uses_identity_when_child_absent():
    overrides = [ov.normalize_override({"child": "ghost_link", "position": [0, 0, 0]})]

    kept, applied = ov.plan_overrides([FakeTransform("map", "vicon")], overrides)

    assert [t.child_frame_id for t in kept] == ["vicon"]
    assert applied[0]["replaced"] == 0
    assert applied[0]["rotation_source"] == "identity"
    assert applied[0]["quat"] == pytest.approx(ov.IDENTITY_QUAT)


def test_explicit_rotation_wins_over_recorded():
    transforms = [FakeTransform("map", "cam", (0.0, 0.0, 0.0, 1.0))]
    overrides = [ov.normalize_override({
        "child": "cam", "position": [1, 2, 3],
        "orientation": {"roll_deg": 0, "pitch_deg": 0, "yaw_deg": 180},
    })]

    _, applied = ov.plan_overrides(transforms, overrides)

    assert applied[0]["rotation_source"] == "override"
    assert applied[0]["quat"] == pytest.approx((0.0, 0.0, 1.0, 0.0), abs=1e-9)
