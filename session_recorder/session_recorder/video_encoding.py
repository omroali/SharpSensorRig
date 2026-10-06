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

Software fallback (2026-10-01 incident)
---------------------------------------
The rig used to hardcode NVENC. After the host's NVIDIA driver/library pair
drifted (``Failed to initialize NVML: Driver/library version mismatch``,
``cuInit(0) failed -> CUDA_ERROR_COMPAT_NOT_SUPPORTED_ON_DEVICE``) every ffmpeg
process died during encoder init and all four videos came out at 0 bytes.

:func:`probe_encoder` runs the same tiny encode the operator pre-flight uses;
:func:`resolve_encoder` falls back to libx264/libx265 when the requested
hardware encoder cannot initialise, and reports failure when nothing works.
The probe and the decision are pure functions with an injectable ``run`` so this
policy is testable on a workstation with no GPU (and, as of writing, no working
NVIDIA driver).
"""

from __future__ import annotations

import re
import subprocess
from dataclasses import dataclass, field
from typing import Callable

#: Target keyframe spacing in seconds. One per second keeps seeks cheap (<= one
#: second of decode) for a negligible bitrate cost.
KEYFRAME_INTERVAL_S = 1.0

#: Encoders that consume frames as NV12 and use NVENC-style quality flags.
NVENC_ENCODERS = {"hevc_nvenc", "h264_nvenc"}

#: CPU encoders we are prepared to fall back to.
SOFTWARE_ENCODERS = {"libx264", "libx265"}

#: hardware encoder -> software equivalent used when the probe fails.
SOFTWARE_FALLBACKS = {
    "h264_nvenc": "libx264",
    "hevc_nvenc": "libx265",
    "h264_qsv": "libx264",
    "hevc_qsv": "libx265",
    "h264_amf": "libx264",
    "hevc_amf": "libx265",
}

#: x264/x265 speed preset for the fallback. ``veryfast`` keeps four concurrent
#: 960x540/1080p streams inside the rig's CPU budget; x264's default ``medium``
#: cannot keep up and would drop frames through the recorder's queue.
SOFTWARE_PRESETS = {"libx264": "veryfast", "libx265": "veryfast"}

#: Input side of the encoder probe. Matches the operator pre-flight command
#: (``bash_scripts/container/check_encoder.sh``) as ``ffmpeg -f lavfi -i
#: nullsrc=s=256x256:d=0.1 -c:v <encoder> -f null -``.
#:
#: The frame size must stay above NVENC's minimum encode dimension. A 64x64
#: probe -- what the 2026-10-01 remediation plan originally specified -- makes
#: NVENC fail with "InitializeEncoder failed: invalid param (8): Frame Dimension
#: less than the minimum supported value" even when the GPU is perfectly
#: healthy, so the recorder would log a spurious NVENC failure and fall back to
#: the CPU encoder for no reason. 256x256 is comfortably clear of the limit.
PROBE_INPUT = ("-f", "lavfi", "-i", "nullsrc=s=256x256:d=0.1")

#: A dead NVENC usually fails in well under a second; the timeout only exists so
#: a wedged probe cannot block rig startup or the start of a recording.
PROBE_TIMEOUT_S = 10.0


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
    else:
        preset = SOFTWARE_PRESETS.get(encoder)
        if preset:
            args += ["-preset", preset]
    args += quality_flags(encoder, crf)
    # Must come after -tune so it overrides the preset's infinite GOP.
    args += ["-g", str(keyframe_interval(mux_fps))]
    return args


def software_fallback_for(encoder: str) -> str | None:
    """CPU encoder to use when ``encoder`` cannot initialise (None if unknown)."""
    return SOFTWARE_FALLBACKS.get(str(encoder or ""))


def is_software_encoder(encoder: str) -> bool:
    return str(encoder or "") in SOFTWARE_ENCODERS


@dataclass(frozen=True)
class ProbeResult:
    """Outcome of one ``ffmpeg -c:v <encoder> -f null -`` probe."""

    encoder: str
    ok: bool
    returncode: int | None = None
    detail: str = ""

    def summary(self) -> str:
        if self.ok:
            return f"{self.encoder}: OK"
        return f"{self.encoder}: FAILED ({self.detail or 'no diagnostics'})"


@dataclass(frozen=True)
class EncoderDecision:
    """Which encoder a stream will actually use, and why."""

    requested: str
    effective: str
    ok: bool
    fallback_used: bool = False
    detail: str = ""

    def summary(self) -> str:
        if self.ok and not self.fallback_used:
            return f"{self.requested}: OK"
        if self.ok:
            return (
                f"{self.requested}: UNAVAILABLE ({self.detail or 'probe failed'}) "
                f"-> falling back to {self.effective}"
            )
        return (
            f"{self.requested}: NO WORKING ENCODER "
            f"({self.detail or 'probe failed and no fallback'}); video WILL be empty"
        )


def probe_command(encoder: str) -> list[str]:
    """The exact command used to prove ``encoder`` can initialise."""
    return [
        "ffmpeg",
        "-hide_banner",
        "-nostdin",
        "-loglevel",
        "error",
        *PROBE_INPUT,
        "-c:v",
        str(encoder),
        "-f",
        "null",
        "-",
    ]


def _last_error_line(text: str) -> str:
    """Pick the line that names the cause, not ffmpeg's generic trailing one."""
    lines = [line.strip() for line in (text or "").splitlines() if line.strip()]
    if not lines:
        return ""
    cause = re.compile(
        r"cuda|nvml|unknown encoder|no such|not (found|supported)|cannot|could not|failed",
        re.IGNORECASE,
    )
    for line in lines:
        if cause.search(line):
            return line
    return lines[-1]


def probe_encoder(
    encoder: str,
    *,
    timeout_s: float = PROBE_TIMEOUT_S,
    run: Callable[..., "subprocess.CompletedProcess[str]"] = subprocess.run,
) -> ProbeResult:
    """Encode 0.1 s of 64x64 black and report whether ``encoder`` initialised.

    This is the same check the operator runs before a session (B4); it is done
    here so the recorder can choose a working encoder instead of writing 0-byte
    files. Never raises: a probe failure is data, not an exception.
    """
    encoder = str(encoder or "")
    if not encoder:
        return ProbeResult(encoder, False, None, "no encoder configured")
    cmd = probe_command(encoder)
    try:
        result = run(cmd, capture_output=True, text=True, timeout=timeout_s)
    except FileNotFoundError:
        return ProbeResult(encoder, False, None, "ffmpeg not found on PATH")
    except subprocess.TimeoutExpired:
        return ProbeResult(encoder, False, None, f"probe timed out after {timeout_s:.0f}s")
    except OSError as exc:  # pragma: no cover - defensive
        return ProbeResult(encoder, False, None, f"probe could not run: {exc}")
    stderr = (result.stderr or "").strip()
    if result.returncode == 0:
        return ProbeResult(encoder, True, 0, "")
    return ProbeResult(
        encoder,
        False,
        result.returncode,
        _last_error_line(stderr) or f"ffmpeg exited {result.returncode}",
    )


def resolve_encoder(
    requested: str,
    *,
    probe: Callable[[str], ProbeResult] = probe_encoder,
    allow_software_fallback: bool = True,
) -> EncoderDecision:
    """Return the encoder to use for ``requested``, probing hardware first.

    ``ok=False`` means neither the requested encoder nor its software fallback
    could initialise: the caller must alarm, because any video written by
    ``requested`` will be empty.
    """
    requested = str(requested or "")
    if not requested:
        return EncoderDecision("", "", False, False, "no encoder configured")
    primary = probe(requested)
    if primary.ok:
        return EncoderDecision(requested, requested, True, False, "")
    fallback = software_fallback_for(requested) if allow_software_fallback else None
    if not fallback or fallback == requested:
        return EncoderDecision(requested, requested, False, False, primary.detail)
    secondary = probe(fallback)
    if secondary.ok:
        return EncoderDecision(requested, fallback, True, True, primary.detail)
    return EncoderDecision(
        requested,
        requested,
        False,
        False,
        f"{primary.detail or 'failed'}; {fallback}: {secondary.detail or 'failed'}",
    )


@dataclass
class EncoderResolver:
    """Probe-once encoder policy shared by every stream of one recorder node.

    One probe per distinct encoder (not per stream): four colour streams must not
    pay four NVENC init failures, and the status payload below is what the
    recording manager echoes onto the operator's console.
    """

    requested: str
    allow_software_fallback: bool = True
    probe: Callable[[str], ProbeResult] = probe_encoder
    _cache: dict[str, ProbeResult] = field(default_factory=dict, repr=False)
    _decisions: dict[str, EncoderDecision] = field(default_factory=dict, repr=False)

    def probe_result(self, encoder: str) -> ProbeResult:
        encoder = str(encoder or "")
        if encoder not in self._cache:
            self._cache[encoder] = self.probe(encoder)
        return self._cache[encoder]

    def resolve(self, requested: str, stream: str) -> EncoderDecision:
        decision = resolve_encoder(
            requested,
            probe=self.probe_result,
            allow_software_fallback=self.allow_software_fallback,
        )
        self._decisions[str(stream)] = decision
        return decision

    @property
    def fallback_used(self) -> bool:
        return any(d.fallback_used for d in self._decisions.values())

    @property
    def decisions(self) -> dict[str, EncoderDecision]:
        return dict(self._decisions)

    @property
    def broken_streams(self) -> list[str]:
        return [stream for stream, d in self._decisions.items() if not d.ok]

    @property
    def effective_encoders(self) -> list[str]:
        """Encoders that will actually be used (broken streams excluded)."""
        return sorted(
            {d.effective for d in self._decisions.values() if d.ok and d.effective}
        )

    def status_payload(self, checked_at: str) -> dict:
        return {
            "checked_at": checked_at,
            "requested": self.requested,
            "software_fallback": bool(self.allow_software_fallback),
            "fallback_used": self.fallback_used,
            "effective_encoders": self.effective_encoders,
            "broken_streams": self.broken_streams,
            "probes": {
                encoder: {"ok": r.ok, "detail": r.detail}
                for encoder, r in self._cache.items()
            },
            "streams": {
                stream: {
                    "requested": d.requested,
                    "effective": d.effective,
                    "ok": d.ok,
                    "fallback_used": d.fallback_used,
                    "detail": d.detail,
                }
                for stream, d in self._decisions.items()
            },
        }
