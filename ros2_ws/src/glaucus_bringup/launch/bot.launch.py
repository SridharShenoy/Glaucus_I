"""Launch everything that runs ON THE ROBOT (Raspberry Pi).

    ros2 launch glaucus_bringup bot.launch.py
    ros2 launch glaucus_bringup bot.launch.py camera:=false   # bench test without camera

Pi setup notes (2 GB RAM):
  - Ubuntu Server 24.04 + ROS 2 Jazzy (ros-jazzy-ros-base, NOT desktop). No GUI.
  - Expect roughly: OS ~300 MB, each C++ node ~20-60 MB, camera_ros ~80 MB.
    Plenty of headroom as long as SLAM stays topside.
  - Camera: camera_ros (libcamera) publishes /camera/image_raw for vision_node.
    The topside gets /camera/image_raw/compressed via image_transport.
    Pi 4 has a hardware H.264 encoder; Pi 5 does not (JPEG/software H.264 then).
    If CPU gets tight, drop resolution to 640x480 @ 15 fps first.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params = PathJoinSubstitution([FindPackageShare("glaucus_bringup"), "config", "sweep_params.yaml"])
    camera = LaunchConfiguration("camera")
    port = LaunchConfiguration("mcu_port")

    return LaunchDescription([
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("mcu_port", default_value="/dev/ttyACM0"),

        Node(package="glaucus_bot", executable="mcu_bridge_node", name="mcu_bridge",
             parameters=[{"port": port}], respawn=True, respawn_delay=1.0),
        Node(package="glaucus_bot", executable="sweep_executor_node", name="sweep_executor",
             parameters=[params]),
        Node(package="glaucus_bot", executable="vision_node", name="vision_node",
             condition=IfCondition(camera)),
        Node(package="camera_ros", executable="camera_node", name="camera",
             namespace="camera", condition=IfCondition(camera),
             parameters=[{"width": 640, "height": 480, "format": "RGB888"}]),
        # TODO(glaucus): lock exposure / white balance (see vision_node.cpp LIGHTING note)
    ])
