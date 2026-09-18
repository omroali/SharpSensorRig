#!/usr/bin/env python3
"""Re-encode a session's colour videos so scrubbing stays fast.

Sessions recorded before the ``-g`` fix have a single I-frame per video: the
NVENC low-latency preset (``-preset p1 -tune ll``) defaults to an *infinite*
GOP. H.264/H.265 random access starts decoding at the previous keyframe, so on
those files every scrub seeks back to frame 0 — the further into the session you
jump, the longer it takes ("the scrubber gets slower and slower"). Playback is
unaffected because it reads sequentially.

This rewrites each ``<session>/videos/*.mp4`` in place with a fresh keyframe
every second, preserving the frame count so the existing ``*.csv`` timestamp
tables stay valid. Files that already have periodic keyframes are skipped.

Usage:
  python3 repair_video_keyframes.py <session_dir> [--encoder auto|h264_nvenc|libx264]
                                    [--crf 18] [--dry-run] [--force]

Requires ffmpeg/ffprobe on PATH and, for one file at a time, free disk space
roughly equal to that file's size.
"""

from __future__ import annotations

import argparse
import glob
import os
import shutil
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from session_recorder.video_encoding import build_encoder_args, keyframe_interval  # noqa: E402


def _probe(path: str, entries: str, extra: list[str] | None = None) -> str:
    cmd = ["ffprobe", "-v", "error", "-select_streams", "v:0"]
    cmd += extra or []
    cmd += ["-show_entries", entries, "-of", "default=noprint_wrappers=1:nokey=1", path]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"ffprobe failed on {path}: {result.stderr.strip()}")
    return result.stdout.strip()


def video_fps(path: str) -> float:
    """Nominal frame rate as a float (r_frame_rate is a fraction like 2949/100)."""
    raw = _probe(path, "stream=r_frame_rate").splitlines()[0]
    num, _, den = raw.partition("/")
    return float(num) / float(den) if den else float(num)


def frame_count(path: str) -> int | None:
    raw = _probe(path, "stream=nb_frames").splitlines()[0]
    return int(raw) if raw.isdigit() else None


def keyframe_count(path: str) -> int:
    """Count I-frames; ``-skip_frame nokey`` makes this a fast partial decode."""
    result = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "v:0", "-skip_frame", "nokey",
         "-show_entries", "frame=pict_type", "-of", "csv=p=0", path],
        capture_output=True, text=True,
    )
    return sum(1 for line in result.stdout.splitlines() if line.strip() == "I")


def _encode(src: str, dst: str, encoder: str, crf: int, fps: float) -> bool:
    cmd = ["ffmpeg", "-v", "error", "-y", "-i", src]
    cmd += build_encoder_args(encoder, crf, fps)
    cmd += ["-fps_mode", "passthrough", "-movflags", "+faststart", dst]
    return subprocess.run(cmd).returncode == 0


def repair(path: str, args) -> bool:
    """Re-encode one video in place. Returns True if it was rewritten."""
    size = os.path.getsize(path)
    fps = video_fps(path)
    frames = frame_count(path)
    keyframes = keyframe_count(path)

    if keyframes > 1 and not args.force:
        print(f"  skip {os.path.basename(path)}: already has {keyframes} keyframes")
        return False

    gop = keyframe_interval(fps)
    print(f"  {os.path.basename(path)}: {frames} frames @ {fps:.2f} fps, "
          f"{keyframes} keyframe(s) -> re-encoding with gop={gop} "
          f"({size / 1e6:.0f} MB)")
    if args.dry_run:
        return False

    tmp = path + ".keyfix.tmp.mp4"
    encoders = [args.encoder] if args.encoder != "auto" else ["h264_nvenc", "libx264"]
    try:
        for encoder in encoders:
            print(f"    encoding with {encoder} …", flush=True)
            if _encode(path, tmp, encoder, args.crf, fps):
                break
            print(f"    {encoder} failed; trying next")
        else:
            print(f"    ERROR: no encoder succeeded for {path}", file=sys.stderr)
            return False

        new_frames = frame_count(tmp)
        new_keys = keyframe_count(tmp)
        if frames is not None and new_frames != frames:
            print(f"    ERROR: frame count changed {frames} -> {new_frames}; "
                  f"leaving original untouched", file=sys.stderr)
            return False
        if new_keys <= 1:
            print(f"    ERROR: re-encode still has {new_keys} keyframe(s); "
                  f"leaving original untouched", file=sys.stderr)
            return False

        os.replace(tmp, path)
        print(f"    done: {new_keys} keyframes, {new_frames} frames")
        return True
    finally:
        if os.path.exists(tmp):
            os.remove(tmp)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("session_dir", help="Session directory (containing videos/)")
    parser.add_argument("--encoder", default="auto",
                        help="auto (try h264_nvenc then libx264), or an ffmpeg encoder name")
    parser.add_argument("--crf", type=int, default=18, help="Constant-quality level (default 18)")
    parser.add_argument("--dry-run", action="store_true", help="Report only, do not encode")
    parser.add_argument("--force", action="store_true",
                        help="Re-encode even if the file already has keyframes")
    args = parser.parse_args(argv)

    if shutil.which("ffmpeg") is None or shutil.which("ffprobe") is None:
        print("ERROR: ffmpeg/ffprobe not found on PATH", file=sys.stderr)
        return 2

    videos = sorted(glob.glob(os.path.join(args.session_dir, "videos", "*.mp4")))
    if not videos:
        print(f"ERROR: no videos/*.mp4 under {args.session_dir}", file=sys.stderr)
        return 2

    print(f"{len(videos)} video(s) in {args.session_dir}/videos")
    changed = 0
    for video in videos:
        if repair(video, args):
            changed += 1
    print(f"{'Would rewrite' if args.dry_run else 'Rewrote'} {changed} of {len(videos)} video(s).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
