#!/usr/bin/env python3
"""Launch the core demo stack without the interactive voice terminal."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_model = LaunchConfiguration("robot_model")
    robot_family = LaunchConfiguration("robot_family")
    launch_bridge = LaunchConfiguration("launch_bridge")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_model", default_value="kr6_r900_2"),
            DeclareLaunchArgument("robot_family", default_value="agilus"),
            DeclareLaunchArgument("launch_bridge", default_value="false"),
            Node(
                package="kuka_ros2_demo",
                executable="control_server",
                name="control_server",
                output="screen",
            ),
            Node(
                package="kuka_ros2_demo",
                executable="vision_node",
                name="vision_node",
                output="screen",
                parameters=[
                    {
                        "homography_path": "/home/emil/kuka_ros2/src/kuka_ros2_demo/data/aruco_homography.npy"
                    }
                ],
            ),
            Node(
                package="kuka_ros2_demo",
                executable="pick_place_coordinator",
                name="pick_place_coordinator",
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("kuka_resources"),
                        "/launch/fake_hardware_planning_template.launch.py",
                    ]
                ),
                launch_arguments={
                    "robot_model": robot_model,
                    "robot_family": robot_family,
                    "dof": "6",
                    "moveit_config": "kr",
                }.items(),
            ),
            Node(
                package="kuka_eki_bridge",
                executable="gripper_bridge",
                name="gripper_bridge",
                output="screen",
                condition=IfCondition(launch_bridge),
            ),
        ]
    )
