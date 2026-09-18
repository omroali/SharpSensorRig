#!/bin/bash

# setup_and_build_ws.sh
# This script sets up the ROS 2 workspace, builds it, and sources it.

# Define workspace path from environment variable (set in Dockerfile/docker-compose.yml)
BASE_WS=${BASE_WS:-/home/ros/base_ws} # Use default if not set

# Check if BASE_WS exists
if [ ! -d "${BASE_WS}" ]; then
    echo "Error: BASE_WS directory '${BASE_WS}' not found. Exiting setup script."
    return 1 # Use return for sourcing script, exit for direct execution
fi

# Navigate to the workspace root
cd "${BASE_WS}" || { echo "Error: Could not change to BASE_WS directory '${BASE_WS}'."; return 1; }

echo "Navigated to ROS 2 workspace: $(pwd)"

# Ensure host-mounted librealsense2 libs are in the linker cache
sudo ldconfig 2>/dev/null || true
export LD_LIBRARY_PATH="/usr/local/lib:$LD_LIBRARY_PATH"

# Check if setup.bash already exists (meaning it might have been built before)
# and only build if it hasn't or if a rebuild is explicitly requested.
if [ ! -f "install/setup.bash" ] || [ "$1" == "--rebuild" ]; then
    # Clean stale build artifacts to prevent cmake cache corruption / SIGSEGV
    if [ -d "build" ] || [ -d "install" ]; then
        echo "Cleaning previous build artifacts..."
        rm -rf build install log
        echo "Cleaned."
    fi

    echo "Running colcon build..."
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Drealsense2_DIR=/usr/local/lib/cmake/realsense2
    rm -f install/COLCON_IGNORE  # prevent blocking ament discovery
    if [ $? -ne 0 ]; then
        echo "Error: colcon build failed! Please check the build output above."
        echo "To retry, run: wbuild --rebuild"
    fi
else
    echo "Workspace already built (install/setup.bash found). Skipping colcon build."
    echo "To force a rebuild, run 'wbuild --rebuild'."
fi

REALSENSE_CONFIG="$HOME/.realsense-config.json"
if [ ! -f "$REALSENSE_CONFIG" ]; then
    echo 'Creating RealSense DDS config...'
    cat > "$REALSENSE_CONFIG" << 'REALEOF'
{
  "context": {
    "dds": {
      "domain": 0,
      "enabled": true
    }
  }
}
REALEOF
fi

# Source the ROS 2 base and workspace environment
echo "Sourcing ROS 2 base environment..."
source /opt/ros/jazzy/setup.bash

echo "Sourcing workspace environment..."
source ${BASE_WS}/install/setup.bash

echo "ROS 2 workspace setup and sourced. Happy robot wrangling!"

alias launch_kinect="ros2 launch kinect2_bridge multi_kinect.launch.py"
alias launch_recording="ros2 launch session_recorder unified_recording.launch.py"
alias launch_realsense="ros2 launch realsense_tf_broadcaster realsense_multi_camera.launch.py"

# vicon
alias launch_vicon="ros2 launch vicon_receiver all.launch.py"
alias calibrate_cameras="ros2 run kinect2_bridge vicon_marker_calibration_tf.py"

# velodyne (device IP + pose come from $SENSOR_CONFIG_DIR/velodyne.yaml)
alias launch_velodyne="ros2 launch velodyne velodyne_with_tf.launch.py"

alias start="ros2 service call /start_recording std_srvs/srv/Trigger"
alias stop="ros2 service call /stop_recording std_srvs/srv/Trigger"

# Replay a recorded session (bag + videos + point clouds + RViz):
#   replay <session_dir>   e.g. replay ~/data/<uuid>/session_20260909_123226
alias replay="ros2 run session_recorder replay_session"

# Label frames with activities from /sharp/events, then scrub/jump in RViz:
#   label_session <session_dir>   # writes activities.yaml + labels/
#   scrub <session_dir>           # scrub + jump between activities
alias label_session="ros2 run session_recorder session_activities"
alias scrub="ros2 run session_recorder session_player"

# Both replay tools move recorded frames if the session carries a
# <session>/tf_overrides.yaml (or if given --tf-override CHILD=[PARENT:]X,Y,Z).
# Use it when a frame was published at the wrong pose during recording --
# the bad transform is baked into the bag and a config edit cannot undo it.
#   e.g. realsense_d55_1_link=-3.4535,-0.4038,1.0216

alias launch_all="bash $HOME/bash_scripts/tmux_launch.sh"
alias terminate_all="bash $HOME/bash_scripts/tmux_terminate.sh"


alias refresh_usb="bash $HOME/bash_scripts/refresh_usb.sh"

# Force a clean rebuild (deletes build/ install/ log/ first)
alias wclean="rm -rf ${BASE_WS}/build ${BASE_WS}/install ${BASE_WS}/log && echo 'Workspace cleaned.'"
alias wbuild="cd ${BASE_WS} && wclean && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 && source ${BASE_WS}/install/setup.bash"

# Mark that the setup has been run in this shell session
export _ROS_WORKSPACE_SETUP_RUN=true
