#!/usr/bin/env python3
"""ffmpeg encoder arguments shared by the video recorders (pure, unit-tested).

Kept separate from :mod:`colour_video_recorder` so the argument construction can
be tested without a ROS runtime.

Keyframe policy
---------------
H.264/H.265 random access starts decoding at the previous keyframe (I-frame), so
the GOP length is the worst-case cost of one seek. The NVENC preset this project
uses (``-preset p1 -tune ll``) defaults to an *infinite* GOP — a single I-frame
for the whole clip — which makes every scrub decode from frame 0 and get slower
the further into a session you go. We therefore always pass an explicit ``-g``;
NVENC honours ``avctx->gop_size`` over the preset (see FFmpeg ``nvenc.c``
``nvenc_setup_encoder``).
"""

from __future__ import annotations

#: Target keyframe spacing in seconds. One per second keeps seeks cheap (<= one
#: second of decode) for a negligible bitrate cost.
KEYFRAME_INTERVAL_S = 1.0

#: Encoders that consume frames as NV12 and use NVENC-style quality flags.
NVENC_ENCODERS = {"hevc_nvenc", "h264_nvenc"}


def keyframe_interval(mux_fps: float, seconds: float = KEYFRAME_INTERVAL_S) -> int:
    """GOP length in frames giving at least one keyframe per ``seconds``."""
    if mux_fps <= 0:
        return 1
    return max(1, int(round(mux_fps * seconds)))


def quality_flags(encoder: str, crf: int) -> list[str]:
    """Rate-control flags for ``encoder`` (constant-quality style)."""
    if encoder in NVENC_ENCODERS:
        return ["-cq", str(crf)]
    if encoder == "hevc_qsv":
        return ["-global_quality", str(crf)]
    if encoder == "hevc_amf":
        return ["-qp_i", str(crf), "-qp_p", str(crf)]
    return ["-crf", str(crf)]


def build_encoder_args(encoder: str, crf: int, mux_fps: float) -> list[str]:
    """Output-side ffmpeg arguments: codec, quality, and a bounded GOP."""
    output_fmt = "nv12" if encoder in NVENC_ENCODERS else "yuv420p"
    args = ["-vcodec", encoder, "-vf", f"format={output_fmt}"]
    if encoder in NVENC_ENCODERS:
        args += ["-preset", "p1", "-tune", "ll"]
    args += quality_flags(encoder, crf)
    # Must come after -tune so it overrides the preset's infinite GOP.
    args += ["-g", str(keyframe_interval(mux_fps))]
    return args
