"""Launch everything that runs TOPSIDE (laptop / mini PC that is also the Wi-Fi AP).

    ros2 launch glaucus_bringup topside.launch.py
    ros2 launch glaucus_bringup topside.launch.py record:=true   # ALWAYS do this in the tank

The web gateway is started separately (it's a plain Python app that also
speaks ROS):  python webapp/gateway/server.py --ros
"""
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    record = LaunchConfiguration("record")
    bag_name = f"bags/glaucus_{datetime.now():%Y%m%d_%H%M%S}"

    return LaunchDescription([
        DeclareLaunchArgument("record", default_value="false"),

        Node(package="glaucus_topside", executable="mission_manager", name="mission_manager"),
        Node(package="glaucus_topside", executable="coverage_mapper", name="coverage_mapper"),
        # Node(package="glaucus_topside", executable="hull_mapper"),   # stretch goal

        # Bags are the single most useful debugging tool you have. Record
        # everything except raw images (compressed only, or the disk fills).
        ExecuteProcess(
            condition=IfCondition(record),
            cmd=["ros2", "bag", "record", "-o", bag_name, "--storage", "mcap",
                 "/glaucus/telemetry", "/glaucus/evidence", "/glaucus/drive_cmd",
                 "/glaucus/sweep_status", "/glaucus/sweep_events", "/glaucus/coverage",
                 "/camera/image_raw/compressed"],
            output="screen"),
    ])
