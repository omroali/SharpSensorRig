"""Unit tests for session replay discovery (session_recorder.replay_session).

Pure-function tests: no ROS runtime required. Run with:
  python3 -m pytest session_recorder/test/test_replay_session.py
"""

import os
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from session_recorder import replay_session as replay  # noqa: E402


def _write_session(tmp_path):
    """Synthesise a session with two colour videos and mixed depth topics."""
    session = tmp_path / "session_20260909_123226"
    videos = session / "videos"
    bag = session / "bag"
    videos.mkdir(parents=True)
    bag.mkdir()

    expected = {
        "session": {"participant": "t", "min_rate_factor": 0.5},
        "expected_topics": [
            {"topic": "/kinect2_1/qhd/image_color_rect/compressed", "sink": "video"},
            {"topic": "/realsense/D555_419222301842/color/image_raw/compressed", "sink": "video"},
            {"topic": "/kinect2_1/sd/image_depth", "sink": "bag"},
            {"topic": "/kinect2_1/sd/camera_info", "sink": "bag"},
        ],
    }
    (session / "expected_topics.yaml").write_text(yaml.safe_dump(expected))

    for stem in (
        "kinect2_1_qhd_image_color_rect_compressed",
        "realsense_D555_419222301842_color_image_raw_compressed",
    ):
        (videos / f"{stem}.mp4").write_bytes(b"")
        (videos / f"{stem}.csv").write_text("frame_idx,ros_timestamp_ns\n0,1\n")
    # A video with no manifest entry, still discovered via fallback.
    (videos / "orphan_stream.mp4").write_bytes(b"")
    (videos / "orphan_stream.csv").write_text("frame_idx,ros_timestamp_ns\n")

    def meta(topic, type_):
        return {"topic_metadata": {"name": topic, "type": type_}, "message_count": 1}

    metadata = {
        "rosbag2_bagfile_information": {
            "topics_with_message_count": [
                meta("/kinect2_1/sd/image_depth", "sensor_msgs/msg/Image"),
                meta("/kinect2_1/sd/camera_info", "sensor_msgs/msg/CameraInfo"),
                meta("/kinect2_1/sd/image_ir/throttled", "sensor_msgs/msg/Image"),
                meta("/kinect2_1/sd/image_depth/compressed", "sensor_msgs/msg/CompressedImage"),
                meta(
                    "/realsense/D555_419222301842/depth/image_rect_raw/throttled",
                    "sensor_msgs/msg/Image",
                ),
                meta(
                    "/realsense/D555_419222301842/depth/camera_info",
                    "sensor_msgs/msg/CameraInfo",
                ),
                # Depth with no camera_info in the bag -> must be skipped.
                meta("/kinect2_2/sd/image_depth", "sensor_msgs/msg/Image"),
                meta("/velodyne_1/velodyne_points", "sensor_msgs/msg/PointCloud2"),
                meta("/tf", "tf2_msgs/msg/TFMessage"),
                meta("/vicon/markers/Sensors_B/Top1", "geometry_msgs/msg/PointStamped"),
                meta("/vicon/unlabeled_markers/marker_0", "geometry_msgs/msg/PointStamped"),
                meta("/vicon/markers_visualization", "visualization_msgs/msg/MarkerArray"),
            ]
        }
    }
    (bag / "metadata.yaml").write_text(yaml.safe_dump(metadata))
    return session


# --- discovery -------------------------------------------------------------

def test_find_bag_dir(tmp_path):
    session = _write_session(tmp_path)
    assert replay.find_bag_dir(str(session)) == os.path.join(str(session), "bag")


def test_discover_videos_maps_manifest_topics_and_strips_compressed(tmp_path):
    session = _write_session(tmp_path)
    specs = replay.discover_videos(str(session), replay.load_expected(str(session)))
    by_publish = {s["publish_topic"]: s for s in specs}

    assert "/kinect2_1/qhd/image_color_rect" in by_publish
    assert "/realsense/D555_419222301842/color/image_raw" in by_publish
    # Manifest-less video is included on a synthetic topic.
    assert "/orphan_stream" in by_publish
    assert len(specs) == 3
    kinect = by_publish["/kinect2_1/qhd/image_color_rect"]
    assert kinect["video"].endswith("kinect2_1_qhd_image_color_rect_compressed.mp4")
    assert kinect["csv"].endswith("kinect2_1_qhd_image_color_rect_compressed.csv")


def test_discover_depths_pairs_camera_info_and_skips_others(tmp_path):
    session = _write_session(tmp_path)
    bag_dir = replay.find_bag_dir(str(session))
    depths = replay.discover_depths(replay.load_bag_topics(bag_dir))
    by_image = {d["image"]: d for d in depths}

    # Kinect raw depth (unthrottled) + RealSense throttled depth.
    assert set(by_image) == {
        "/kinect2_1/sd/image_depth",
        "/realsense/D555_419222301842/depth/image_rect_raw/throttled",
    }
    assert by_image["/kinect2_1/sd/image_depth"]["camera_info"] == "/kinect2_1/sd/camera_info"
    assert by_image["/kinect2_1/sd/image_depth"]["points"] == "/kinect2_1/sd/points"
    rs = by_image["/realsense/D555_419222301842/depth/image_rect_raw/throttled"]
    assert rs["camera_info"] == "/realsense/D555_419222301842/depth/camera_info"
    assert rs["points"] == "/realsense/D555_419222301842/depth/points"
    # /kinect2_2 has no camera_info; IR and CompressedImage must not appear.
    assert "/kinect2_2/sd/image_depth" not in by_image


def test_discover_extra_topics_includes_tf_and_vicon_but_not_visualization(tmp_path):
    session = _write_session(tmp_path)
    topics = replay.load_bag_topics(replay.find_bag_dir(str(session)))
    extras = replay.discover_extra_topics(topics)
    assert "/tf" in extras
    assert "/vicon/markers/Sensors_B/Top1" in extras
    assert "/vicon/unlabeled_markers/marker_0" in extras
    # The regenerable MarkerArray visualisations stay out, as on the rig.
    assert "/vicon/markers_visualization" not in extras


# --- commands --------------------------------------------------------------

def test_bag_play_cmd_flags():
    assert replay.bag_play_cmd("/b", 1.0, False) == ["ros2", "bag", "play", "/b", "--clock"]
    assert "--loop" in replay.bag_play_cmd("/b", 1.0, True)
    assert "--rate" in replay.bag_play_cmd("/b", 0.5, False)


def test_video_publisher_cmd_arrays():
    specs = [
        {"video": "/v/a.mp4", "csv": "/v/a.csv", "publish_topic": "/a"},
        {"video": "/v/b.mp4", "csv": "/v/b.csv", "publish_topic": "/b"},
    ]
    cmd = replay.video_publisher_cmd(specs)
    assert "videos:=[/v/a.mp4, /v/b.mp4]" in cmd
    assert "timestamp_csvs:=[/v/a.csv, /v/b.csv]" in cmd
    assert "topics:=[/a, /b]" in cmd


def test_pointcloud_cmd_remaps():
    cmd = replay.pointcloud_cmd(
        {"image": "/c/depth/image_rect_raw", "camera_info": "/c/depth/camera_info", "points": "/c/depth/points"}
    )
    assert "camera_info:=/c/depth/camera_info" in cmd
    assert "image_rect:=/c/depth/image_rect_raw" in cmd
    assert "points:=/c/depth/points" in cmd
    assert "use_sim_time:=true" in cmd


def test_rviz_config_has_display_for_every_stream(tmp_path):
    session = _write_session(tmp_path)
    bag_dir = replay.find_bag_dir(str(session))
    videos = replay.discover_videos(str(session), replay.load_expected(str(session)))
    depths = replay.discover_depths(replay.load_bag_topics(bag_dir))

    config = replay.build_rviz_config(videos, depths)
    displays = config["Visualization Manager"]["Displays"]
    classes = [d["Class"] for d in displays]
    image_topics = {d["Topic"]["Value"] for d in displays if d["Class"].endswith("/Image")}
    cloud_topics = {d["Topic"]["Value"] for d in displays if d["Class"].endswith("/PointCloud2")}

    assert config["Visualization Manager"]["Global Options"]["Fixed Frame"] == "map"
    assert classes.count("rviz_default_plugins/Image") == len(videos) + len(depths)
    assert classes.count("rviz_default_plugins/PointCloud2") == len(depths)
    # Every colour topic and every depth image has a display.
    assert {s["publish_topic"] for s in videos} <= image_topics
    assert {d["image"] for d in depths} <= image_topics
    assert {d["points"] for d in depths} == cloud_topics


def test_rviz_config_adds_point_display_per_marker_topic():
    config = replay.build_rviz_config([], [], ["/vicon/markers/Sensors_B/Top1"])
    displays = config["Visualization Manager"]["Displays"]
    points = [d for d in displays if d["Class"] == "rviz_default_plugins/PointStamped"]
    assert len(points) == 1
    assert points[0]["Topic"]["Value"] == "/vicon/markers/Sensors_B/Top1"


def test_rviz_config_status_panel_is_opt_in():
    without = replay.build_rviz_config([], [], [])
    assert not any(
        p.get("Class") == "session_recorder_rviz/SessionStatusPanel"
        for p in without["Panels"]
    )

    with_panel = replay.build_rviz_config([], [], [], status_panel=True)
    panels = [p for p in with_panel["Panels"]
              if p.get("Class") == "session_recorder_rviz/SessionStatusPanel"]
    assert len(panels) == 1
    assert panels[0]["Topic"] == "/session/status_text"


def test_write_rviz_config_roundtrips(tmp_path):
    session = _write_session(tmp_path)
    bag_dir = replay.find_bag_dir(str(session))
    videos = replay.discover_videos(str(session), replay.load_expected(str(session)))
    depths = replay.discover_depths(replay.load_bag_topics(bag_dir))
    out = tmp_path / "nested" / "replay.rviz"

    replay.write_rviz_config(str(out), videos, depths)
    assert out.is_file()
    loaded = yaml.safe_load(out.read_text())
    assert loaded["Visualization Manager"]["Global Options"]["Fixed Frame"] == "map"
