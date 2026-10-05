# On-rig checklist — storage/reliability rework

Everything below was implemented and unit-tested off-rig. These are the
steps that need actual sensor/hardware access to confirm, plus what to watch
during the first participant session.

## Pre-flight — run before every session (added 2026-10-05)

The 2026-10-01 session lost its bag (tmux segfault, unfinalised mcap) and all
four videos (NVENC could not initialise, 0-byte files). These are the checks
that prove the fixes on the real rig. Nothing here needs the sensors.

### 0.1 Encoder path — the one-line check

Inside the rig container (`record` pane is fine), or from the host checkout:

```bash
# inside the container:
bash /home/ubuntu/bash_scripts/check_encoder.sh

# or from SharpSensorRig/ on the host:
docker compose -f docker/docker-compose.yml run --rm sharp-sensor-rig \
    bash /home/ubuntu/bash_scripts/check_encoder.sh
```

Proves success: the last line is `RESULT: HARDWARE OK` (NVENC works) or
`RESULT: SOFTWARE FALLBACK` with `libx264`/`libx265` (CPU encoding — video is
still recorded, expect higher CPU and larger files). Either is usable;
`RESULT: FAIL` means every `.mp4` would be 0 bytes — do not start a session.

The host GPU stack is currently broken (`nvidia-smi` → NVML/driver/library
version mismatch, NVML 580.178), so **software fallback is expected** until the
driver is realigned. Realigning it needs `sudo` and a reboot:

```bash
# host, needs sudo and a reboot — not done from this repo
sudo apt install --reinstall nvidia-driver-<version>   # match `modinfo nvidia | head`
sudo reboot
nvidia-smi                                             # must list the GPU
```

### 0.2 What the recording should show from now on

- `/start_recording` prints the split policy, e.g.
  `bag splitting: max 1024 MiB / 300 s per split`.
- Within ~5 s the console either says `WATCHDOG: video check OK - N stream(s)
  writing non-empty files`, or names the offending stream. If NVENC is down you
  also get `WATCHDOG: VIDEO ENCODER FALLBACK ACTIVE: …`.
- `<session>/bag/` contains `bag_0.mcap`, `bag_1.mcap`, … once a split rolls
  over (5 minutes), plus one `metadata.yaml` covering them all.
- `<session>/recorder_heartbeat.json` is rewritten every ~5 s while recording
  (this is what a host-side watchdog can poll), and `<session>/watchdog.log`
  records alarm/note lines.
- Killing the terminal is now survivable: `SIGHUP`/`SIGTERM` to the manager
  runs the same teardown as `stop`

### 0.3 Three rig-side tests worth 10 minutes (from the re-run gate)

```bash
# 1. Split rollover: record for >5 minutes, then
ls -l <session>/bag/            # expect bag_0.mcap AND bag_1.mcap
yaml=$(ls <session>/bag*/metadata.yaml)  # expect exactly one per bag dir
# 2. Abrupt kill: kill -9 the rosbag2 process mid-recording; every *completed*
#    split must still open (metadata.yaml is written by rosbag2 on graceful
#    stop, so expect it to be missing for the killed run - that is why splits
#    exist; recover with `mcap recover bag_N.mcap -o bag_N_recovered.mcap`).
# 3. Terminal-death test: kill the tmux server, then check the console/log for
#    "Stopping recording (SIGHUP received)" and a session_report.yaml.
```

### 0.4 Container logs now survive the container

`docker compose run --rm` used to take the json-file log with it. The service
now logs to journald, so after the container is gone:

```bash
journalctl -t sharp-rig --since "3 hours ago" | tail -100
```

## 0. Build (both packages changed)

```bash
# C++ bridge changed (tf_static fix) and the python package gained nodes
colcon build --packages-select kinect2_bridge session_recorder
source install/setup.bash
pip install fastapi uvicorn        # health-monitor deps, no rosdep keys
```

## 1. Confirm the deployed config matches the repo  ⚠️ known drift

The 2026-07-10 session recorded only `kinect2_1` + one RealSense, although the
repo config marks `kinect2_3` and both RealSense `record: true`. That means the
container's `$SENSOR_CONFIG_DIR` config differs from `config/` in the repo.

```bash
diff -r "$SENSOR_CONFIG_DIR" "$(git rev-parse --show-toplevel)/config"
```
Reconcile before recording — otherwise a sensor silently won't record.

## 2. Confirm the raw sd topic names exist on this bridge build

The recorder now subscribes to raw sd depth/IR. Verify the bridge advertises
them (it does in this repo's source, but confirm the running build):

```bash
ros2 topic list | grep -E '/kinect2_[0-9]+/sd/(image_depth|image_ir|camera_info)$'
# expect, per camera:
#   /kinect2_X/sd/image_depth
#   /kinect2_X/sd/image_ir
#   /kinect2_X/sd/camera_info
ros2 topic list | grep -E 'compressedDepth'   # realsense usb depth PNG transport
```
If `compressedDepth` is missing, `image_transport` PNG plugin isn't loaded on
the RealSense driver — install `ros-$ROS_DISTRO-image-transport-plugins`.

## 3. Confirm the tf_static fix took effect

Was ~383 Hz (2 transforms × 100 Hz × 2 bridges). Should now be a latched burst
then silence:

```bash
ros2 topic hz /tf_static      # expect it to print the initial msgs then stall
```

## 4. Health monitor

```bash
ros2 launch session_recorder health_monitor.launch.py
# open http://<rig-host>:8765/
htop        # confirm the monitor process stays near-idle during recording
```
Every configured topic should show green at its expected Hz before you record.
Any amber/red row is a sensor to fix now, not after a 3-hour take.

## 5. First session — watch for

- `/start_recording` writes `<session>/expected_topics.yaml` and
  `<session>/calibration/<serial>/` (4 calib_*.yaml per Kinect). Confirm both
  appear.
- `/stop_recording` response ends with `session_check: PASS` (or FAIL + the
  offending topic). Full detail in `<session>/session_report.yaml`.
- Throttle relay log `<session>/topic_throttle.log` shows the IR (5 Hz) and
  RealSense depth (15 Hz) relays connecting.

## 6. Validate the raw→registered depth path once (do this before trusting it)

Record one short take with BOTH raw sd depth and the bridge's registered qhd
depth, to confirm offline regeneration matches. Temporarily add to
`recording.yaml` kinect.streams a second depth entry — or just grab one frame
of each live — then:

```bash
python3 session_recorder/scripts/reproduce_registered_depth.py \
    --calib <session>/calibration/<serial> \
    --depth raw_sd_depth_frame.png --qhd --out regenerated_qhd_depth.png
# compare regenerated_qhd_depth.png against the bridge's image_depth_rect frame
```
The pure-geometry path is covered by `--selftest` (runs off-rig); this step
confirms it against *this* rig's calibration and real data.

## Storage expectation after the rework

~40 GB/h with all sensors healthy → ~120 GB per 3-h participant →
~3.6 TB for 30 participants (fits the 4 TB internal drive with headroom).
If a session comes in materially larger, check that IR/RealSense-depth
throttles actually engaged (`topic_throttle.log`) and that the vicon
`*visualization*` exclude is in effect.
