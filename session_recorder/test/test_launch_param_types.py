"""Static type-consistency check between the launch file and the node.

Why this test exists
--------------------
``ros2 launch session_recorder unified_recording.launch.py`` passed
``bag_max_size_mb`` as a **float** (1024.0) while the node declared it as an
**INTEGER** (from ``hardening.DEFAULT_BAG_MAX_SIZE_MB = 1024``, an int, unlike
its every-other-value-is-a-float siblings). rclpy rejects the mismatch at
construction:

    InvalidParameterTypeException: Trying to set parameter 'bag_max_size_mb'
    to '1024.0' of type 'DOUBLE', expecting type 'INTEGER'

...and the recording manager dies before it can start. The failure is not a
warning: the node never comes up, so a session cannot record at all. It is also
invisible to the unit tests, because it only happens when launch and node meet.

This test parses both files and asserts they agree, so the mismatch surfaces in
CI instead of at the rack.

Run with:
  python3 -m pytest session_recorder/test/test_launch_param_types.py
"""

import os
import re
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from session_recorder import hardening  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH = os.path.join(HERE, "..", "launch", "unified_recording.launch.py")
MANAGER = os.path.join(HERE, "..", "session_recorder", "recording_manager.py")

# rclpy parameter kinds keyed by the Python type the default implies.
KIND_BY_PYTHON_TYPE = {
    int: "INTEGER",
    float: "DOUBLE",
    bool: "BOOL",
    str: "STRING",
}

# The cast the launch file applies, mapped to the kind it produces.
KIND_BY_CAST = {
    "int": "INTEGER",
    "float": "DOUBLE",
    "bool": "BOOL",
    "str": "STRING",
}


def _read(path):
    with open(os.path.abspath(path), encoding="utf-8") as fh:
        return fh.read()


def _literal_kind(expr):
    """Kind implied by a Python literal, or None if not a literal."""
    expr = expr.strip()
    if expr in ("True", "False"):
        return "BOOL"
    if re.fullmatch(r"-?\d+", expr):
        return "INTEGER"
    if re.fullmatch(r"-?\d+\.\d*(?:[eE][-+]?\d+)?", expr) or re.fullmatch(
        r"-?\d+[eE][-+]?\d+", expr
    ):
        return "DOUBLE"
    if expr.startswith(("'", '"')):
        return "STRING"
    # os.path.* helpers all return str (e.g. output_root).
    if expr.startswith("os.path."):
        return "STRING"
    return None


def node_declared_kinds():
    """{parameter name: declared kind} from recording_manager.declare_parameter."""
    text = _read(MANAGER)
    out = {}
    pattern = re.compile(
        r'self\.declare_parameter\(\s*"([^"]+)"\s*,\s*(.+?)\s*\)\s*$',
        re.MULTILINE,
    )
    for name, expr in pattern.findall(text):
        expr = expr.strip()
        if expr == "Parameter.Type.STRING_ARRAY":
            out[name] = "STRING_ARRAY"
            continue
        kind = _literal_kind(expr)
        if kind:
            out[name] = kind
            continue
        # A DEFAULT_* constant imported from hardening.
        const = re.fullmatch(r"([A-Z][A-Z0-9_]*)", expr)
        if const and hasattr(hardening, const.group(1)):
            value = getattr(hardening, const.group(1))
            # bool must be checked before int: bool is a subclass of int.
            for py_type, name_ in (
                (bool, "BOOL"),
                (int, "INTEGER"),
                (float, "DOUBLE"),
                (str, "STRING"),
            ):
                if isinstance(value, py_type):
                    out[name] = name_
                    break
            continue
        # Unparseable: record it so the test can complain rather than skip.
        out[name] = f"UNKNOWN({expr})"
    return out


def launch_casts():
    """{parameter name: kind produced by its launch cast, or None if uncast}.

    Handles both styles used in the launch file: an inline cast
    (``"video_crf": int(...)``) and a cast made earlier into a local
    (``nexus_port = int(...)`` then ``"nexus_port": nexus_port``).
    """
    text = _read(LAUNCH)

    # `var = cast(...)` assignments earlier in the file.
    var_kinds = {}
    for var, cast in re.findall(
        r"^\s*([a-z_][a-z0-9_]*)\s*=\s*([A-Za-z_]+)\(", text, re.MULTILINE
    ):
        if cast in KIND_BY_CAST:
            var_kinds[var] = KIND_BY_CAST[cast]

    out = {}
    for name, value in re.findall(r'"([a-z0-9_]+)":\s*([^,\n]+)', text):
        if name in out:
            continue
        value = value.strip()
        inline = re.match(r"([A-Za-z_]+)\(", value)
        if inline and inline.group(1) in KIND_BY_CAST:
            out[name] = KIND_BY_CAST[inline.group(1)]
        elif value in var_kinds:
            out[name] = var_kinds[value]
        else:
            out[name] = None
    return out


# --- the guard -------------------------------------------------------------

def test_launch_and_node_agree_on_numeric_parameter_types():
    declared = node_declared_kinds()
    casts = launch_casts()

    unknown = {k: v for k, v in declared.items() if str(v).startswith("UNKNOWN")}
    assert not unknown, (
        "could not infer the declared type of these parameters; extend the "
        f"parser rather than leaving them unchecked: {unknown}"
    )

    mismatches = []
    compared = 0
    for name, kind in declared.items():
        if kind not in ("INTEGER", "DOUBLE", "BOOL"):
            continue
        cast_kind = casts.get(name)
        if cast_kind is None:
            # Passed through from YAML without a cast: the value's type then
            # depends on how the YAML happens to be written, which is exactly
            # the fragility this test exists to prevent. Require a cast.
            mismatches.append(
                f"{name}: node declares {kind} but the launch file passes it "
                f"through uncast (add {kind.lower()} cast)"
            )
            continue
        compared += 1
        if cast_kind != kind:
            mismatches.append(
                f"{name}: node declares {kind} but launch casts to {cast_kind}"
            )

    # Guard against the parser silently matching nothing.
    assert compared >= 8, (
        f"only {compared} parameter(s) compared; the parser has probably "
        "drifted from the launch file's formatting"
    )
    assert not mismatches, "launch/node parameter type mismatch:\n  " + "\n  ".join(
        mismatches
    )


def test_bag_max_size_is_an_integer_on_both_sides():
    """The specific regression: 1024 is a MiB count, so INTEGER, not DOUBLE."""
    assert node_declared_kinds()["bag_max_size_mb"] == "INTEGER"
    assert launch_casts()["bag_max_size_mb"] == "INTEGER"


def test_float_declared_siblings_stay_double():
    """bag_max_duration_s and stop_grace_s are declared DOUBLE (300.0 / 30.0)."""
    declared = node_declared_kinds()
    casts = launch_casts()
    for name in ("bag_max_duration_s", "stop_grace_s", "watchdog_interval_s"):
        assert declared[name] == "DOUBLE", name
        assert casts[name] == "DOUBLE", name


if __name__ == "__main__":  # pragma: no cover
    sys.exit(pytest.main([__file__, "-v"]))
