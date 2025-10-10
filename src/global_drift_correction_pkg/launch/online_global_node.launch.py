import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    global_localize = Node(
        package="global_drift_correction_pkg",
        executable="global_relocalization",
        output="screen"
    )

    localize_bridge = Node(
        package="global_drift_correction_pkg",
        executable="localization_bridge",
        output="screen"
    )

    pointcloud_relay = Node(
        package="global_drift_correction_pkg",
        executable="pointcloud_relay",
        output="screen"
    )

    return LaunchDescription([global_localize, localize_bridge, pointcloud_relay])