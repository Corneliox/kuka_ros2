#!/usr/bin/env python3
"""Launch the interactive voice nodes separately from the backend demo stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_voice_terminal = LaunchConfiguration("launch_voice_terminal")

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_voice_terminal", default_value="false"),
            Node(
                package="kuka_ros2_demo",
                executable="voice_ai_node",
                name="voice_ai_node",
                output="screen",
            ),
            Node(
                package="kuka_ros2_demo",
                executable="voice_terminal_mock",
                name="voice_terminal_mock",
                output="screen",
                condition=IfCondition(launch_voice_terminal),
            ),
        ]
    )
