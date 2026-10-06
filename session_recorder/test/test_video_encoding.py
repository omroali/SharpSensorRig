"""Unit tests for the pure ffmpeg-argument helpers.

No ROS runtime required. Run with:
  python3 -m pytest session_recorder/test/test_video_encoding.py
"""

import os
import subprocess
import sys

import pytest

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


# --- software fallback (2026-10-01: NVENC failed, every video was 0 bytes) --

@pytest.mark.parametrize("encoder", ["libx264", "libx265"])
def test_software_encoder_args_add_a_realtime_preset(encoder):
    """x264/x265 default to 'medium', which cannot keep four streams real-time."""
    args = ve.build_encoder_args(encoder, 18, 30.0)
    assert args[args.index("-preset") + 1] == "veryfast"
    assert "nv12" not in args  # NV12 is an NVENC-only input format


def test_software_fallback_map_covers_every_hardware_encoder():
    assert ve.software_fallback_for("h264_nvenc") == "libx264"
    assert ve.software_fallback_for("hevc_nvenc") == "libx265"
    assert ve.software_fallback_for("hevc_qsv") == "libx265"
    assert ve.software_fallback_for("libx264") is None
    assert ve.is_software_encoder("libx264")
    assert not ve.is_software_encoder("h264_nvenc")


# --- the pre-flight probe (remediation plan B4) ----------------------------

def _fake_runner(results):
    """``subprocess.run`` stand-in keyed by the encoder in the probe command."""
    calls = []

    def run(cmd, capture_output=True, text=True, timeout=None):
        del capture_output, text, timeout
        encoder = cmd[cmd.index("-c:v") + 1]
        calls.append(encoder)
        returncode, stderr = results.get(encoder, (1, f"Unknown encoder '{encoder}'"))
        return subprocess.CompletedProcess(cmd, returncode, "", stderr)

    run.calls = calls
    return run


def test_probe_command_matches_the_documented_pre_flight():
    cmd = ve.probe_command("h264_nvenc")
    joined = " ".join(cmd)
    assert cmd[0] == "ffmpeg"
    # 256x256, not 64x64: NVENC refuses frames below its minimum encode
    # dimension, so a 64x64 probe reports FAIL on a healthy GPU and forces a
    # needless fall back to the CPU encoder. See PROBE_INPUT in video_encoding.
    assert "-f lavfi -i nullsrc=s=256x256:d=0.1" in joined
    assert "-c:v h264_nvenc" in joined
    assert cmd[-3:] == ["-f", "null", "-"]


def test_probe_frame_is_large_enough_for_nvenc():
    """Guard the frame size against a well-meaning 'make the probe cheaper' edit.

    NVENC's minimum is far below 256, but the original 64x64 probe was under it.
    Anything at or above 145 wide / 49 high is safe for H.264; assert a generous
    floor so a future shrink cannot silently reintroduce the false negative.
    """
    import re

    joined = " ".join(ve.PROBE_INPUT)
    match = re.search(r"nullsrc=s=(\d+)x(\d+)", joined)
    assert match, f"unexpected probe input: {joined}"
    width, height = (int(g) for g in match.groups())
    assert width >= 145 and height >= 145, (
        f"probe frame {width}x{height} may be below NVENC's minimum "
        "encode dimension, which would fail on a healthy GPU"
    )


def test_probe_encoder_reports_success():
    result = ve.probe_encoder("libx264", run=_fake_runner({"libx264": (0, "")}))
    assert result.ok
    assert result.returncode == 0
    assert result.detail == ""


def test_probe_encoder_captures_the_cuda_failure_line():
    stderr = (
        "[h264_nvenc @ 0x1] dl_fn->cuda_dl->cuInit(0) failed -> "
        "CUDA_ERROR_COMPAT_NOT_SUPPORTED_ON_DEVICE\n"
        "Error initializing output stream 0:0\n"
    )
    result = ve.probe_encoder(
        "h264_nvenc", run=_fake_runner({"h264_nvenc": (1, stderr)})
    )
    assert not result.ok
    assert "CUDA_ERROR_COMPAT_NOT_SUPPORTED_ON_DEVICE" in result.detail


def test_probe_encoder_handles_timeout_and_missing_ffmpeg():
    def timeout_runner(cmd, capture_output=True, text=True, timeout=None):
        del cmd, capture_output, text, timeout
        raise subprocess.TimeoutExpired("ffmpeg", 25.0)

    def missing_runner(cmd, capture_output=True, text=True, timeout=None):
        del cmd, capture_output, text, timeout
        raise FileNotFoundError("ffmpeg")

    timed_out = ve.probe_encoder("h264_nvenc", run=timeout_runner)
    assert not timed_out.ok and "timed out" in timed_out.detail
    missing = ve.probe_encoder("h264_nvenc", run=missing_runner)
    assert not missing.ok and "not found" in missing.detail


def test_resolve_encoder_keeps_a_working_hardware_encoder():
    runner = _fake_runner({"h264_nvenc": (0, "")})
    decision = ve.resolve_encoder(
        "h264_nvenc", probe=lambda e: ve.probe_encoder(e, run=runner)
    )
    assert decision.ok and not decision.fallback_used
    assert decision.effective == "h264_nvenc"
    assert runner.calls == ["h264_nvenc"]  # no pointless software probe


def test_resolve_encoder_falls_back_to_libx264_when_nvenc_fails():
    runner = _fake_runner(
        {
            "h264_nvenc": (1, "cuInit(0) failed -> CUDA_ERROR_COMPAT_NOT_SUPPORTED_ON_DEVICE"),
            "libx264": (0, ""),
        }
    )
    decision = ve.resolve_encoder(
        "h264_nvenc", probe=lambda e: ve.probe_encoder(e, run=runner)
    )
    assert decision.ok and decision.fallback_used
    assert decision.effective == "libx264"
    assert "CUDA_ERROR_COMPAT_NOT_SUPPORTED_ON_DEVICE" in decision.detail
    assert "libx264" in decision.summary()


def test_resolve_encoder_reports_when_nothing_works():
    runner = _fake_runner({})  # every probe fails
    decision = ve.resolve_encoder(
        "hevc_nvenc", probe=lambda e: ve.probe_encoder(e, run=runner)
    )
    assert not decision.ok
    assert decision.effective == "hevc_nvenc"
    assert "NO WORKING ENCODER" in decision.summary()
    assert "WILL be empty" in decision.summary()


def test_resolve_encoder_honours_disabled_fallback():
    runner = _fake_runner({"libx264": (0, "")})
    decision = ve.resolve_encoder(
        "h264_nvenc",
        probe=lambda e: ve.probe_encoder(e, run=runner),
        allow_software_fallback=False,
    )
    assert not decision.ok and not decision.fallback_used
    assert runner.calls == ["h264_nvenc"]


# --- resolver used by the recorder node ------------------------------------

def test_resolver_probes_each_encoder_once_across_streams():
    runner = _fake_runner({"h264_nvenc": (0, "")})
    resolver = ve.EncoderResolver(
        requested="hevc_nvenc", probe=lambda e: ve.probe_encoder(e, run=runner)
    )
    for topic in ("/a/color/image_raw", "/b/color/image_raw"):
        decision = resolver.resolve("h264_nvenc", topic)
        assert decision.effective == "h264_nvenc"
    assert runner.calls == ["h264_nvenc"]


def test_resolver_status_payload_is_what_the_manager_reads():
    runner = _fake_runner(
        {
            "h264_nvenc": (1, "cuInit(0) failed"),
            "libx264": (0, ""),
            "hevc_nvenc": (1, "Failed to initialize NVML"),
        }
    )
    resolver = ve.EncoderResolver(
        requested="hevc_nvenc", probe=lambda e: ve.probe_encoder(e, run=runner)
    )
    resolver.resolve("h264_nvenc", "/cam/color/image_raw/compressed")
    resolver.resolve("hevc_nvenc", "/cam/sd/image_ir")
    payload = resolver.status_payload("2026-10-05T00:00:00")

    assert payload["requested"] == "hevc_nvenc"
    assert payload["fallback_used"] is True
    assert payload["effective_encoders"] == ["libx264"]
    assert payload["broken_streams"] == ["/cam/sd/image_ir"]
    assert payload["probes"]["h264_nvenc"]["ok"] is False
    assert (
        payload["streams"]["/cam/color/image_raw/compressed"]["effective"] == "libx264"
    )

