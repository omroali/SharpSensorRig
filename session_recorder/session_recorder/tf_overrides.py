"""Static-transform overrides for session replay.

A recorded session bakes its TF tree into the bag's ``/tf_static``, so a frame
that was published at the wrong pose during recording cannot be corrected by
editing the rig config afterwards — the bad transform is already in the bag.

Overrides let a session declare replacements, either in
``<session>/tf_overrides.yaml``::

    overrides:
      - parent: map                      # optional, defaults to "map"
        child: realsense_d55_1_link
        position: {x: -3.454, y: -0.404, z: 1.022}
        # orientation is optional; omit it to keep the recorded rotation
        # orientation: {roll_deg: 0.0, pitch_deg: 0.0, yaw_deg: -90.0}

or on the command line (repeatable, ``--tf-override``)::

    realsense_d55_1_link=-3.454,-0.404,1.022
    realsense_d55_1_link=map:-3.454,-0.404,1.022@0,0,-90

Every recorded transform whose ``child_frame_id`` matches is dropped, so the
frame ends up with exactly one parent instead of the two that a duplicated
``map -> <camera>`` static publisher creates. When no orientation is supplied
the rotation already recorded for that child is reused, so a translation-only
move never changes where the camera points.

The helpers here are deliberately free of ROS imports (rclpy/rosbag2 are only
imported inside the functions that need them) so they can be unit tested
without a ROS environment, matching the rest of this package.
"""

from __future__ import annotations

import math
import os
from typing import Any, Iterable, Optional, Sequence

import yaml

DEFAULT_PARENT = "map"
OVERRIDES_FILENAME = "tf_overrides.yaml"

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]

IDENTITY_QUAT: Quat = (0.0, 0.0, 0.0, 1.0)


# ── parsing ─────────────────────────────────────────────────────────────────


def _as_vec3(value: Any, what: str) -> Vec3:
    """Accept ``{x,y,z}``, ``[x,y,z]`` or ``(x,y,z)``."""
    if isinstance(value, dict):
        try:
            return (float(value["x"]), float(value["y"]), float(value["z"]))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"{what}: expected x/y/z keys, got {value!r}") from exc
    if isinstance(value, (list, tuple)) and len(value) == 3:
        return (float(value[0]), float(value[1]), float(value[2]))
    raise ValueError(f"{what}: expected 3 numbers, got {value!r}")


def _optional_rpy_deg(entry: dict, what: str) -> Optional[Vec3]:
    """Rotation is optional; ``None`` means "keep whatever was recorded"."""
    for key in ("orientation", "rotation", "rpy_deg"):
        if key in entry and entry[key] is not None:
            value = entry[key]
            if isinstance(value, dict):
                keys = ("roll_deg", "pitch_deg", "yaw_deg")
                if not all(k in value for k in keys):
                    keys = ("roll", "pitch", "yaw")
                try:
                    return (float(value[keys[0]]), float(value[keys[1]]), float(value[keys[2]]))
                except (KeyError, TypeError, ValueError) as exc:
                    raise ValueError(f"{what}: bad orientation {value!r}") from exc
            return _as_vec3(value, f"{what}.orientation")
    return None


def normalize_override(entry: dict) -> dict:
    """Validate one override mapping into a canonical dict."""
    if not isinstance(entry, dict):
        raise ValueError(f"override must be a mapping, got {entry!r}")
    child = entry.get("child") or entry.get("child_frame_id")
    if not child:
        raise ValueError(f"override needs a 'child' frame: {entry!r}")
    parent = entry.get("parent") or entry.get("parent_frame_id") or DEFAULT_PARENT
    position = entry.get("position")
    if position is None:
        if all(k in entry for k in ("x", "y", "z")):
            position = {k: entry[k] for k in ("x", "y", "z")}
        else:
            raise ValueError(f"override for {child} needs a 'position'")
    return {
        "parent": str(parent),
        "child": str(child),
        "position": _as_vec3(position, f"{child}.position"),
        "rotation": _optional_rpy_deg(entry, child),
    }


def parse_override_spec(spec: str) -> dict:
    """Parse ``child=[parent:]x,y,z[@roll,pitch,yaw]`` (degrees)."""
    text = str(spec).strip()
    if "=" not in text:
        raise ValueError(
            f"bad --tf-override {spec!r}; expected CHILD=[PARENT:]X,Y,Z[@R,P,Y]"
        )
    child, _, rest = text.partition("=")
    child = child.strip()
    if not child:
        raise ValueError(f"bad --tf-override {spec!r}: empty child frame")

    rotation: Optional[Vec3] = None
    if "@" in rest:
        rest, _, rot_text = rest.partition("@")
        rotation = _as_vec3([p for p in rot_text.split(",")], f"{child}.orientation")

    parent = DEFAULT_PARENT
    if ":" in rest:
        parent, _, rest = rest.partition(":")
        parent = parent.strip() or DEFAULT_PARENT

    position = _as_vec3([p for p in rest.split(",")], f"{child}.position")
    return {"parent": parent, "child": child, "position": position, "rotation": rotation}


def load_overrides(session_dir: str) -> list[dict]:
    """Read ``<session>/tf_overrides.yaml`` (missing file -> no overrides)."""
    path = os.path.join(session_dir, OVERRIDES_FILENAME)
    if not os.path.isfile(path):
        return []
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    entries = data.get("overrides", []) if isinstance(data, dict) else data
    return [normalize_override(entry) for entry in (entries or [])]


def merge_overrides(*groups) -> list[dict]:
    """Concatenate override lists, last entry per child frame wins.

    Lets a command-line ``--tf-override`` override a session file without
    publishing two competing transforms for the same child frame.
    """
    merged: dict[str, dict] = {}
    for group in groups:
        for entry in group or []:
            merged[entry["child"]] = entry
    return list(merged.values())


# ── maths ───────────────────────────────────────────────────────────────────


def quat_from_rpy_deg(rotation: Vec3) -> Quat:
    """ROS-standard (roll, pitch, yaw) degrees -> quaternion xyzw."""
    roll, pitch, yaw = (math.radians(float(a)) for a in rotation)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _quat_of(transform) -> Quat:
    q = transform.transform.rotation
    return (float(q.x), float(q.y), float(q.z), float(q.w))


def _child_of(transform) -> str:
    return str(transform.child_frame_id)


def _parent_of(transform) -> str:
    return str(transform.header.frame_id)


# ── planning ────────────────────────────────────────────────────────────────


def plan_overrides(
    transforms: Iterable[Any], overrides: Sequence[dict]
) -> tuple[list[Any], list[dict]]:
    """Split recorded static transforms around the overrides.

    Returns ``(kept, applied)`` where ``kept`` is every input transform whose
    child frame is *not* overridden, and ``applied`` is one
    ``{parent, child, position, quat}`` dict per override, ready to be turned
    into a TransformStamped. A recorded rotation is reused (preferring the
    transform with the same parent) unless the override specifies one.
    """
    transforms = list(transforms)
    overridden = {ov["child"] for ov in overrides}

    dropped: dict[str, list[Any]] = {child: [] for child in overridden}
    kept: list[Any] = []
    for transform in transforms:
        child = _child_of(transform)
        if child in overridden:
            dropped[child].append(transform)
        else:
            kept.append(transform)

    applied: list[dict] = []
    for ov in overrides:
        child = ov["child"]
        candidates = dropped.get(child, [])
        recorded_quat: Optional[Quat] = None
        for transform in candidates:
            if _parent_of(transform) == ov["parent"]:
                recorded_quat = _quat_of(transform)
                break
        if recorded_quat is None and candidates:
            recorded_quat = _quat_of(candidates[0])

        if ov.get("rotation") is not None:
            quat = quat_from_rpy_deg(ov["rotation"])
        else:
            quat = recorded_quat or IDENTITY_QUAT

        applied.append({
            "parent": ov["parent"],
            "child": child,
            "position": tuple(float(v) for v in ov["position"]),
            "quat": quat,
            "replaced": len(candidates),
            "rotation_source": (
                "override" if ov.get("rotation") is not None
                else ("recorded" if recorded_quat is not None else "identity")
            ),
        })
    return kept, applied


def overridden_children(overrides: Sequence[dict]) -> set[str]:
    return {ov["child"] for ov in overrides}


def describe_overrides(applied: Sequence[dict]) -> str:
    lines = []
    for entry in applied:
        x, y, z = entry["position"]
        lines.append(
            f"  {entry['parent']} -> {entry['child']}  "
            f"({x:+.4f}, {y:+.4f}, {z:+.4f})  "
            f"[{entry['rotation_source']} rotation, replaced {entry['replaced']} transform(s)]"
        )
    return "\n".join(lines)
