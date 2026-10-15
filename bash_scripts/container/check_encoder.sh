#!/bin/bash
# Pre-flight encoder check for the SHARP sensor rig.
#
# Run this INSIDE the rig container before every participant session, and again
# if the container has been up for days:
#
#   bash /home/ubuntu/bash_scripts/check_encoder.sh
#
# or from the host checkout (from SharpSensorRig/):
#
#   docker compose -f docker/docker-compose.yml run --rm sharp-sensor-rig \
#       bash /home/ubuntu/bash_scripts/check_encoder.sh
#
# It first aligns the container's driver userspace with the host kernel module
# (see align_nvidia_driver_libs.sh -- a version skew there was the real cause of
# the 2026-10-01 video loss), then runs the encoder probe from that plan:
#
#   ffmpeg -f lavfi -i nullsrc=s=256x256:d=0.1 -c:v <encoder> -f null -
#
# NOTE the frame size: NVENC rejects frames below its minimum encode dimension
# ("InitializeEncoder failed: invalid param (8): Frame Dimension less than the
# minimum supported value"). The plan originally specified 64x64, which is below
# that minimum -- so on a perfectly healthy GPU the probe reported FAIL and the
# rig would have fallen back to CPU encoding for no reason. 256x256 is safely
# above the limit and still instantaneous.
#
# for every encoder colour_video_recorder may use, then states which encoder it
# will actually select for this rig's config (config/recording.yaml ->
# video_encoder). The recorder itself runs the same probe at every recording
# start, so this script is the operator-visible half of the same check.
#
# Exit status:
#   0 = at least one encoder can record video (a software fallback still
#       records video; read the RESULT line before starting a session)
#   1 = NO encoder works: every video would be 0 bytes. Do not start a session.

set -u -o pipefail

# Align the container's driver userspace with the host kernel module BEFORE
# probing. A version skew there makes every NVENC probe fail for a reason the
# operator cannot fix from inside the container -- see the header of
# align_nvidia_driver_libs.sh. Non-fatal if it cannot act.
ALIGN_LIBS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/align_nvidia_driver_libs.sh"
if [ -f "$ALIGN_LIBS" ]; then
    bash "$ALIGN_LIBS"
fi

CONFIG="${SENSOR_CONFIG_DIR:-/home/ubuntu/config}/recording.yaml"
PROBE_INPUT=(-f lavfi -i nullsrc=s=256x256:d=0.1)
PROBE_TIMEOUT=30

probe() {
    local encoder="$1"
    local log rc detail
    log="$(mktemp)"
    timeout "${PROBE_TIMEOUT}" ffmpeg -hide_banner -nostdin -loglevel error \
        "${PROBE_INPUT[@]}" -c:v "$encoder" -f null - >"$log" 2>&1
    rc=$?
    if [ "$rc" -eq 0 ]; then
        printf 'PASS  %s\n' "$encoder"
    elif [ "$rc" -eq 124 ]; then
        printf 'FAIL  %s (timed out after %ss)\n' "$encoder" "$PROBE_TIMEOUT"
    else
        # Prefer the first line naming the real cause (the CUDA/NVML error for
        # NVENC, "unknown encoder" for a missing build) over ffmpeg's generic
        # trailing message.
        detail="$(grep -m1 -i -E 'cuda|nvml|unknown encoder|no such|not (found|supported)|cannot|could not' "$log" || true)"
        [ -n "$detail" ] || detail="$(grep -v '^[[:space:]]*$' "$log" | tail -n 1 || true)"
        printf 'FAIL  %s (%s)\n' "$encoder" "$(printf '%s' "$detail" | tr -s '[:space:]' ' ')"
    fi
    rm -f "$log"
    return "$rc"
}

fallback_for() {
    case "$1" in
        h264_nvenc|h264_qsv|h264_amf) echo "libx264" ;;
        hevc_nvenc|hevc_qsv|hevc_amf) echo "libx265" ;;
        *) echo "" ;;
    esac
}

# Probe results, cached for effective_for().
rc_h264_nvenc=1
rc_hevc_nvenc=1
rc_libx264=1
rc_libx265=1

probe_result() {
    case "$1" in
        h264_nvenc) return "$rc_h264_nvenc" ;;
        hevc_nvenc) return "$rc_hevc_nvenc" ;;
        libx264)    return "$rc_libx264" ;;
        libx265)    return "$rc_libx265" ;;
        *)          return 1 ;;
    esac
}

# Mirror video_encoding.resolve_encoder: requested encoder, else its software
# equivalent. Prints the encoder or nothing.
effective_for() {
    local want="$1" fb
    if probe_result "$want"; then
        echo "$want"
        return 0
    fi
    fb="$(fallback_for "$want")"
    if [ -n "$fb" ] && probe_result "$fb"; then
        echo "$fb"
        return 0
    fi
    echo ""
    return 1
}

echo "=== SHARP rig encoder pre-flight ($(date -Is 2>/dev/null || date)) ==="
echo "container ffmpeg: $(ffmpeg -hide_banner -version 2>/dev/null | head -n 1)"

if command -v nvidia-smi >/dev/null 2>&1; then
    if nvidia-smi -L >/dev/null 2>&1; then
        echo "GPU: $(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null | head -n 1)"
    else
        echo "GPU: nvidia-smi FAILED (NVML/driver mismatch) - NVENC cannot work; a software encoder is required"
    fi
else
    echo "GPU: nvidia-smi not installed"
fi

echo
echo "--- encoder probes: ffmpeg ${PROBE_INPUT[*]} -c:v <encoder> -f null - ---"
probe h264_nvenc; rc_h264_nvenc=$?
probe hevc_nvenc; rc_hevc_nvenc=$?
probe libx264;    rc_libx264=$?
probe libx265;    rc_libx265=$?

requested="hevc_nvenc"
if [ -f "$CONFIG" ]; then
    configured="$(sed -n 's/^[[:space:]]*video_encoder:[[:space:]]*"\?\([A-Za-z0-9_]*\)"\?.*/\1/p' "$CONFIG" | head -n 1)"
    if [ -n "$configured" ]; then
        requested="$configured"
    fi
    echo
    echo "config: $CONFIG -> video_encoder: $requested"
else
    echo
    echo "config: $CONFIG not found; assuming video_encoder: $requested"
fi

# Colour topics are recorded as H.264 even when the config asks for hevc_nvenc
# (colour_video_recorder maps hevc_nvenc -> h264_nvenc for /color/ and _Color).
colour_requested="$requested"
case "$requested" in
    hevc_*) colour_requested="h264_nvenc" ;;
esac

colour_encoder="$(effective_for "$colour_requested" || true)"
other_encoder="$(effective_for "$requested" || true)"

echo
if [ -z "$colour_encoder" ] && [ -z "$other_encoder" ]; then
    echo "RESULT: FAIL - no working encoder. Every video would be 0 bytes."
    echo "        Fix the host GPU stack or install a build of ffmpeg with libx264/libx265."
    exit 1
fi

if [ "$colour_encoder" != "$colour_requested" ] || [ "$other_encoder" != "$requested" ]; then
    echo "RESULT: SOFTWARE FALLBACK - the recorder will use:"
    echo "        colour streams : ${colour_encoder:-NONE (video would be empty)}"
    echo "        other streams  : ${other_encoder:-NONE (video would be empty)}"
    echo "        Video is still recorded, on the CPU. Expect higher CPU load and larger files."
    echo "        To restore NVENC: realign the NVIDIA driver/kernel module on the host"
    echo "        (sudo required, then reboot) and re-run this check."
    exit 0
fi

echo "RESULT: HARDWARE OK - the recorder will use '$requested'."
exit 0
