"""Unit tests for the pure ffmpeg-argument helpers.

No ROS runtime required. Run with:
  python3 -m pytest session_recorder/test/test_video_encoding.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from session_recorder import video_encoding as ve  # noqa: E402


# --- keyframe_interval -----------------------------------------------------

def test_keyframe_interval_is_one_per_second():
    assert ve.keyframe_interval(30.0) == 30
    assert ve.keyframe_interval(14.49) == 14
    assert ve.keyframe_interval(29.49) == 29


def test_keyframe_interval_never_below_one():
    assert ve.keyframe_interval(0.0) == 1
    assert ve.keyframe_interval(-5.0) == 1
    assert ve.keyframe_interval(0.2) == 1


def test_keyframe_interval_honours_seconds_override():
    assert ve.keyframe_interval(30.0, seconds=2.0) == 60


# --- build_encoder_args ----------------------------------------------------

def _value_after(args, flag):
    return args[args.index(flag) + 1]


def test_nvenc_args_always_carry_an_explicit_gop():
    """Regression: without -g, -tune ll makes NVENC use an infinite GOP, so
    every scrub decoded from frame 0 and got slower the further in you went."""
    args = ve.build_encoder_args("h264_nvenc", 18, 30.0)
    assert "-g" in args
    assert _value_after(args, "-g") == "30"
    # -g must follow -tune, which is what sets the infinite-GOP preset.
    assert args.index("-tune") < args.index("-g")


def test_nvenc_args_match_previous_behaviour_plus_gop():
    args = ve.build_encoder_args("h264_nvenc", 18, 30.0)
    assert args[:2] == ["-vcodec", "h264_nvenc"]
    assert _value_after(args, "-vf") == "format=nv12"
    assert _value_after(args, "-preset") == "p1"
    assert _value_after(args, "-tune") == "ll"
    assert _value_after(args, "-cq") == "18"


def test_libx264_args_use_crf_and_yuv420p():
    args = ve.build_encoder_args("libx264", 20, 15.0)
    assert _value_after(args, "-vcodec") == "libx264"
    assert _value_after(args, "-vf") == "format=yuv420p"
    assert _value_after(args, "-crf") == "20"
    assert _value_after(args, "-g") == "15"


def test_quality_flags_per_encoder():
    assert ve.quality_flags("h264_nvenc", 1) == ["-cq", "1"]
    assert ve.quality_flags("hevc_nvenc", 1) == ["-cq", "1"]
    assert ve.quality_flags("hevc_qsv", 1) == ["-global_quality", "1"]
    assert ve.quality_flags("hevc_amf", 1) == ["-qp_i", "1", "-qp_p", "1"]
    assert ve.quality_flags("libx265", 1) == ["-crf", "1"]
