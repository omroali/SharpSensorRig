#!/bin/bash

SESSION="ros2"
WS_SETUP="source /opt/ros/jazzy/setup.bash && source ~/base_ws/install/setup.bash"

launch_window() {
    local name="$1"
    local cmd="$2"
    tmux new-window -t "$SESSION" -n "$name" bash -i
    sleep 2  # let .bashrc finish before typing
    tmux send-keys -t "$SESSION:$name" "$cmd" Enter
}

tmux has-session -t "$SESSION" 2>/dev/null && {
    echo "Session '$SESSION' already exists. Attach: tmux attach -t $SESSION"
    exit 0
}

# Kinect
tmux new-session -d -s "$SESSION" -n "kinect" bash -i
sleep 2  # let .bashrc finish before typing
tmux send-keys -t "$SESSION:kinect" "ros2 launch kinect2_bridge multi_kinect.launch.py launch_rviz:=false launch_delay_sec:=0 launch_point_clouds:=false" Enter

# RealSense (cameras + TF, driven by realsense_config.yaml)
launch_window "realsense" "ros2 launch realsense_tf_broadcaster realsense_multi_camera.launch.py"

# Vicon
launch_window "vicon" "ros2 launch vicon_receiver all.launch.py"

# Calibration
launch_window "calib" "ros2 run kinect2_bridge vicon_marker_calibration_tf.py"

# Velodyne (device IP + pose come from $SENSOR_CONFIG_DIR/velodyne.yaml)
launch_window "velodyne" "ros2 launch velodyne velodyne_with_tf.launch.py"

# RViz
launch_window "rviz" "rviz2 -d \$(ros2 pkg prefix kinect2_bridge)/share/kinect2_bridge/launch/kinect_viz.rviz"

# Unified recording manager (service-driven — use 'start'/'stop' aliases to control)
launch_window "record" "ros2 launch session_recorder unified_recording.launch.py"

# Horizontal split below the recording manager with usage instructions.
tmux split-window -v -t "$SESSION:record" bash -i
# sleep 2  # let .bashrc finish before typing
# tmux send-keys -t "$SESSION:record" "echo \"to begin recording, type 'start' to end recording, type 'stop'\"" Enter


launch_window "health" "ros2 run session_recorder topic_health_monitor"

tmux select-window -t "$SESSION:0"
tmux attach -t "$SESSION"
