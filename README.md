# Unified Multi-Sensor Data Collection Rig

Multi-sensor data-collection rig for PhD research, running ROS 2 Jazzy inside a
Docker container. Sensors:

- 2× **Kinect v2** (USB)
- 2× **RealSense D555** (PoE, optional USB via `driver: usb`)
- 1× **Vicon** motion-capture system
- 1× **Velodyne VLP-16** LiDAR(s)

All sensors are configured from a single `config/` directory. Recording is
controlled by one `/start_recording` → `/stop_recording` service pair. Each take
lands in a single session folder with H.265 videos and a zstd-compressed mcap bag.

---

## Host prerequisites

- **Docker** and **docker compose**
- **NVIDIA Container Toolkit** (for NVENC video encoding)
- **librealsense2** installed on the host in `/usr/local/lib` (mounted into the
  container)
- **udev rules** for Kinect devices — run `sudo device_setup.sh` once
- Recording disk mounted at `/media/oali/RECORDING_DATA`

---

## Quick start

```bash
# 1. Start the container
cd docker && docker compose up -d

# 2. Enter the container
docker exec -it docker-sharp-sensor-rig-1 bash

# 3. First time only — build the ROS workspace
wbuild

# 4. Launch all sensors (tmux windows: kinect | realsense | vicon | calib | velodyne | record)
launch_all

# 5. Start / stop recording
start       # begin recording
stop        # end recording
```

Alternatively, use the recording dashboard:

```bash
bash ~/bash_scripts/record.sh
```

---

## Where data lands

On the host: `/media/oali/RECORDING_DATA` (mounted at `~/data` inside the
container).

```
/media/oali/RECORDING_DATA/<uuid>/session_<timestamp>/
├── videos/
│   ├── <topic>.mkv          # H.265 video per colour/IR stream
│   └── <topic>.csv          # per-frame timestamps
└── bag/
    └── bag_0.mcap           # zstd-compressed: depth, camera_info, velodyne, vicon, TF
```

---

## Configuration

See [`config/README.md`](config/README.md) for the full table of which file feeds
which launch. Quick reference:

| File | Purpose |
|------|---------|
| `kinect_cameras.yaml` | Kinect serials, poses, record flags, Vicon calibration objects |
| `realsense_cameras.yaml` | RealSense serials, IP (PoE), `driver: dds\|usb`, poses |
| `velodyne.yaml` | Velodyne IP + pose (live-reloaded on file save — edit and the TF updates) |
| `recording.yaml` | Encoder, CRF, bag compression, per-stream toggles, vicon/velodyne topic lists |

### RealSense driver modes

Each RealSense entry accepts `driver: dds` (PoE, default) or `driver: usb`
(USB-connected — requires `ros-jazzy-realsense2-camera` installed in the
container). The launch system handles both transparently.

---

## Adding a sensor

1. Add an entry to the appropriate YAML in `config/` (with `enabled: true` and
   `record: true`)
2. If the sensor has new topic patterns, extend
   `session_recorder/session_recorder/topics.py`
3. Add a tmux window in `bash_scripts/container/tmux_launch.sh` and an alias in
   `bash_scripts/container/bashrc_extensions.sh`

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| NVENC not available | Ensure `video` is in `NVIDIA_DRIVER_CAPABILITIES` in `docker-compose.yml` |
| Kinect not detected after unplug/replug | Run `refresh_usb` in the container |
| Velodyne point cloud at origin | The TF node (`static_tf_from_yaml.py`) isn't running — check the `velodyne` tmux window (it's part of `velodyne_with_tf.launch.py`). Verify `/velodyne_1/velodyne_points` is publishing. |
| Second Velodyne launches but publishes nothing | Both units are on the same UDP port; give each a unique destination port in its web interface and in `velodyne.yaml`. |
| `realsense_recorder` import errors after rebuild | Run `rm -rf build/realsense_recorder install/realsense_recorder` and rebuild |
