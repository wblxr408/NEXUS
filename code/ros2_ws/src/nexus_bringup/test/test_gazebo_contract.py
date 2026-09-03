import math
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[5]


def test_generated_world_uses_imx219_profile_and_ros_topics():
    profile = yaml.safe_load((REPOSITORY_ROOT / "simulation" / "imx219_gazebo_camera.yaml").read_text())
    world = ET.parse(REPOSITORY_ROOT / "simulation" / "nexus_sandbox_imx219.world")
    camera = world.find(".//sensor[@name='imx219']/camera")
    assert camera is not None
    assert camera.findtext("image/width") == str(profile["camera"]["image_width_px"])
    assert camera.findtext("image/height") == str(profile["camera"]["image_height_px"])
    assert math.isclose(float(camera.findtext("horizontal_fov")), math.radians(79.3), rel_tol=1e-9)
    plugin = world.find(".//sensor[@name='imx219']/plugin")
    sensor = world.find(".//sensor[@name='imx219']")
    assert sensor.findtext("pose") == "0 0 -0.06 0 1.57079632679 0"
    assert plugin.findtext("ros/namespace") == "/nexus/camera"
    assert plugin.findtext("camera_name") == "imx219"
    assert plugin.findtext("frame_name") == "nexus_uav_camera_optical_frame"


def test_uav_is_movable_and_exposes_one_odom_source():
    world = ET.parse(REPOSITORY_ROOT / "simulation" / "nexus_sandbox_imx219.world")
    uav = world.find(".//model[@name='nexus_uav']")
    assert uav.findtext("static") == "false"
    assert uav.findtext("link/gravity") == "false"
    move_plugin = uav.find("plugin[@filename='libgazebo_ros_planar_move.so']")
    assert move_plugin is not None
    assert move_plugin.findtext("ros/namespace") == "/nexus/gazebo/uav"
    assert move_plugin.findtext("ros/remapping") == "cmd_vel:=cmd_vel"
    assert move_plugin.findtext("publish_odom") == "true"
    assert uav.find("plugin[@filename='libgazebo_ros_p3d.so']") is None


def test_simulation_broadcasts_the_single_map_to_base_link_edge():
    """The one publisher of map -> base_link, without which the chain is dead.

    coord_transform_node looks this edge up per sample and answers
    transform_unavailable when it is missing, so /nexus/target/pose never
    appears.  pre_hardware_transforms.yaml must not also publish it statically.
    """
    world = ET.parse(REPOSITORY_ROOT / "simulation" / "nexus_sandbox_imx219.world")
    move_plugin = world.find(".//model[@name='nexus_uav']/plugin[@filename='libgazebo_ros_planar_move.so']")
    assert move_plugin.findtext("publish_odom_tf") == "true"
    assert move_plugin.findtext("odometry_frame") == "map"
    assert move_plugin.findtext("robot_base_frame") == "base_link"
    transforms = yaml.safe_load((REPOSITORY_ROOT / "code/ros2_ws/src/nexus_bringup/config/pre_hardware_transforms.yaml").read_text())
    assert all(item["child"] != "base_link" for item in transforms["transforms"])
