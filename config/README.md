# Unified sensor configuration

Single source of truth for every sensor in the rig. This directory is
mounted into the container at `/home/ubuntu/config` and exported as
`$SENSOR_CONFIG_DIR`; all launch files look here first and fall back to
their installed package copies if a file is missing.

| File | Consumed by |
|------|-------------|
| `kinect_cameras.yaml` | `multi_kinect.launch.py`, `vicon_marker_calibration_tf.py`, `unified_recording.launch.py` |
| `realsense_cameras.yaml` | `realsense_multi_camera.launch.py`, `unified_recording.launch.py` |
| `velodyne.yaml` | `velodyne_with_tf.launch.py` (one entry per unit with unique `port`, driver IP + map→velodyne pose, topic namespaced by entry name), `unified_recording.launch.py` |
| `recording.yaml` | `unified_recording.launch.py` (encoder + fallback probe, CRF, bag storage/compression/splitting, watchdog, per-stream toggles, vicon, velodyne) |

Edits take effect on the next node (re)launch — no rebuild required.
Exception: the velodyne pose is live — `static_tf_from_yaml.py` watches
`velodyne.yaml` and re-publishes the transform when the file changes.

## Crash-resilience keys in `recording.yaml`

Added after the 2026-10-01 session lost a 72-minute bag and four 0-byte videos.

| Key | Default | Effect |
|-----|---------|--------|
| `bag_max_duration_s` | `300` | rosbag2 closes and finalises a split every 5 min; a crash costs at most one split (`0` = unlimited) |
| `bag_max_size_mb` | `1024` | additional size cap per split (`0` = unlimited) |
| `video_software_fallback` | `true` | fall back to `libx264`/`libx265` when the hardware encoder will not initialise |
| `video_encoder_probe` | `true` | probe the encoder at rig start with the pre-flight encode and print the verdict |
| `video_startup_grace_s` | `20` | wait before calling a missing/0-byte `.mp4` a failure on the console |
| `watchdog.enabled` / `interval_s` | `true` / `5` | poll the recorder children for the whole session, write `recorder_heartbeat.json` |
| `watchdog.restart_bag` | `true` | restart a dead bag recorder into a fresh `bag_N/` (previous splits are kept) |
| `stop_grace_s` | `30` | drain window for a graceful stop (must stay below compose `stop_grace_period: 60s`) |

