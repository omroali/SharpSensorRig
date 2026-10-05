#!/usr/bin/env bash
#
# Build the ROS 2 workspace in a way that survives the compiler instability on
# this host.
#
# gcc-13.3 (Ubuntu 24.04) intermittently ICEs while compiling this workspace,
# as a nondeterministic Segmentation fault inside cc1plus. Two distinct crash
# sites have been observed:
#
#   * iterative_hash_template_arg while parsing kinect2_registration
#     (depth_registration_cpu.cpp, which pulls <Eigen/Geometry> in before rclcpp)
#   * variably_modified_type_p during GIMPLE pass: einline while building
#     gtest_vendor for velodyne_pointcloud
#
# The faults are load/layout dependent -- the same translation unit has been
# seen to fail anywhere between 0% and 55% of the time, and disabling ASLR
# makes the first site fail 100% of the time. They are NOT specific to gcc-13:
# the same translation unit also ICEs with gcc 11, 12 and 14, so no compiler
# version or flag combination removes them.
#
# Two mitigations are applied here:
#
#   * --parallel-workers caps how many packages compile at once, keeping peak
#     memory use down. MAKEFLAGS=-j1 already serialises the make steps within
#     each package; this caps colcon's package-level parallelism too, which
#     otherwise defaults to the core count.
#   * retrying re-runs colcon, which resumes incrementally, so a retry only
#     costs the translation units that have not compiled yet. Retries happen
#     only for compiler ICEs; a genuine compile error fails immediately.
#
# A full clean build of all 14 packages has been observed to pass with the
# defaults below.
#
# Environment overrides: BUILD_WORKERS, BUILD_ATTEMPTS.
#
# Usage:
#   build_ws.sh            # incremental build
#   build_ws.sh --clean    # wipe build/ install/ log/ first
#
set -u

BASE_WS="${BASE_WS:-$HOME/base_ws}"
# The ICEs are load dependent, so a serial build is less likely to trip them;
# lower BUILD_WORKERS if attempts keep failing.
WORKERS="${BUILD_WORKERS:-2}"
# colcon resumes incrementally, so a retry only costs the translation units that
# have not compiled yet. Three separate ICE sites have been observed (frontend
# template hashing, the early inliner, and the GC marker), so budget generously.
ATTEMPTS="${BUILD_ATTEMPTS:-20}"

# Optional GCC garbage-collector tuning. The GC-marking ICE suggests GC timing
# matters; raising the heap makes collections rarer. Uncomment to experiment.
# export CXXFLAGS_EXTRA="--param ggc-min-expand=100 --param ggc-min-heapsize=131072"

# Keep the compiler/env aligned with docker-compose.yml, which sets the same
# values for the runtime shell.
export CC="${CC:-/usr/bin/gcc-13}"
export CXX="${CXX:-/usr/bin/g++-13}"
# Works around a separate GCC ICE: "double free or corruption during GIMPLE
# pass: fre".
export CFLAGS="${CFLAGS:--fno-tree-fre}"
export CXXFLAGS="${CXXFLAGS:--fno-tree-fre}"
export MAKEFLAGS="${MAKEFLAGS:--j1}"

cd "$BASE_WS" || exit 1

echo "Building ${BASE_WS} with ${CXX} (workers=${WORKERS}, attempts=${ATTEMPTS})"

if [ "${1:-}" = "--clean" ]; then
    echo "Cleaning previous build artifacts..."
    rm -rf build install log
fi

CMAKE_ARGS=(
    --cmake-args
    -DCMAKE_BUILD_TYPE=Release
    -Drealsense2_DIR=/usr/local/lib/cmake/realsense2
)

set -o pipefail

for attempt in $(seq 1 "$ATTEMPTS"); do
    echo "=== colcon build attempt ${attempt}/${ATTEMPTS} (--parallel-workers ${WORKERS}) ==="
    log="$(mktemp)"

    if colcon build --symlink-install --parallel-workers "$WORKERS" \
            "${CMAKE_ARGS[@]}" 2>&1 | tee "$log"; then
        rm -f "$log"
        rm -f install/COLCON_IGNORE  # prevent blocking ament discovery
        echo "=== colcon build succeeded on attempt ${attempt}/${ATTEMPTS} ==="
        exit 0
    fi

    if grep -qE "internal compiler error|Segmentation fault" "$log"; then
        echo "=== attempt ${attempt}/${ATTEMPTS} hit a compiler ICE; retrying ==="
        rm -f "$log"
    else
        echo "=== attempt ${attempt}/${ATTEMPTS} failed for a non-ICE reason; not retrying ===" >&2
        rm -f "$log"
        exit 1
    fi
done

echo "=== colcon build failed after ${ATTEMPTS} attempts ===" >&2
exit 1
