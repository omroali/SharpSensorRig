#!/bin/bash
# Align the container's NVIDIA driver userspace libraries with the running host
# kernel module.
#
# THE PROBLEM (diagnosed 2026-10-06, after a full day lost to it)
# ---------------------------------------------------------------
# docker/Dockerfile installs `nvidia-cuda-toolkit` for the libfreenect2 CUDA
# build (-DENABLE_CUDA=ON). On Ubuntu 24.04 that pulls in Ubuntu's
# `libnvidia-compute-580`, which ships driver userspace **580.178.04**.
# This host runs the Pop!_OS driver, whose kernel module and userspace are
# **580.173.02**.
#
# The NVIDIA container runtime injects the HOST's libcuda/libnvidia-ml into the
# container alongside the image's copies -- both end up present. But the runtime
# does NOT overwrite a symlink the image already created, so:
#
#     libcuda.so.1      -> libcuda.so.580.178.04      (image  -- WRONG)
#     libcuda.so.580.173.02                           (host   -- correct, unused)
#
# The loader therefore uses a userspace newer than the kernel module. NVML
# reports "Driver/library version mismatch" and libcuda's forward-compatibility
# path fails, because CUDA forward compatibility is not supported on consumer
# GeForce hardware -- an RTX 3080 here. ffmpeg surfaces it as:
#
#     cuInit(0) failed -> CUDA_ERROR_COMPAT_NOT_SUPPORTED_ON_DEVICE:
#                         forward compatibility was attempted on non supported HW
#
# Reinstalling the host driver cannot fix this: the mismatch lives in the image.
#
# WHAT THIS SCRIPT DOES
# ---------------------
# Reads the running kernel module version from /proc/driver/nvidia/version (NOT
# nvidia-smi, which is exactly what is broken in this state), then repoints each
# driver library's `.so.1` symlink at the matching versioned file that the
# runtime injected. Idempotent, safe to run repeatedly, and non-fatal: if the
# matching library is absent it leaves everything alone and exits 0, so the
# software encoder fallback still works.
#
# Usage:  bash align_nvidia_driver_libs.sh [--quiet]
#
# Run automatically by check_encoder.sh and tmux_launch.sh.

set -u -o pipefail

QUIET=0
[ "${1:-}" = "--quiet" ] && QUIET=1

say() { [ "$QUIET" -eq 1 ] || printf '%s\n' "$*"; }

LIBDIR="/usr/lib/x86_64-linux-gnu"
VERSION_FILE="/proc/driver/nvidia/version"

# --- the running kernel module version -------------------------------------
if [ ! -r "$VERSION_FILE" ]; then
    say "nvidia-libs: $VERSION_FILE not readable — not a GPU container, nothing to do"
    exit 0
fi

DRIVER_VER="$(grep -oE '[0-9]+\.[0-9]+\.[0-9]+' "$VERSION_FILE" | head -n1)"
if [ -z "$DRIVER_VER" ]; then
    say "nvidia-libs: could not parse a version from $VERSION_FILE — leaving libs alone"
    exit 0
fi

# --- do we need root to change anything? -----------------------------------
SUDO=""
if [ "$(id -u)" -ne 0 ]; then
    if [ -w "$LIBDIR" ]; then
        SUDO=""
    elif command -v sudo >/dev/null && sudo -n true 2>/dev/null; then
        SUDO="sudo -n"
    else
        say "nvidia-libs: cannot write $LIBDIR and no passwordless sudo — leaving libs alone"
        exit 0
    fi
fi

# --- repoint every driver lib that has a matching versioned file -----------
# Restricted to files literally named lib*.so.<kernel-module-version>, so we can
# only ever point a symlink at a library built for the running module.
changed=0
checked=0
missing=0

for target in "$LIBDIR"/lib*.so."$DRIVER_VER"; do
    [ -e "$target" ] || continue
    base="$(basename "$target")"                 # libcuda.so.580.173.02
    stem="${base%.so."$DRIVER_VER"}"             # libcuda
    link="$LIBDIR/$stem.so.1"

    # Only touch symlinks that already exist: if the image never created one,
    # the runtime's own wiring is correct and we leave it alone.
    [ -L "$link" ] || continue

    checked=$((checked + 1))
    current="$(readlink "$link")"
    if [ "$current" = "$base" ]; then
        continue
    fi

    # A dangling previous target is fine; we are replacing it.
    if $SUDO ln -sfn "$base" "$link"; then
        say "nvidia-libs: $stem.so.1  $current  ->  $base  (now matches kernel module $DRIVER_VER)"
        changed=$((changed + 1))
    else
        say "nvidia-libs: FAILED to repoint $link at $base"
    fi
done

if [ "$checked" -eq 0 ]; then
    # No versioned file for this module version: the runtime injected nothing.
    # That is the normal case when there is no GPU, and it is not an error.
    say "nvidia-libs: no driver libraries for kernel module $DRIVER_VER in $LIBDIR — nothing to align"
    exit 0
fi

if [ "$changed" -eq 0 ]; then
    say "nvidia-libs: already aligned with kernel module $DRIVER_VER ($checked link(s) checked)"
else
    say "nvidia-libs: realigned $changed of $checked link(s) to kernel module $DRIVER_VER"
fi

exit 0
