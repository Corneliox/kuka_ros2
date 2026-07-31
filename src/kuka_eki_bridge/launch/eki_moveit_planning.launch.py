"""
eki_moveit_planning.launch.py

Real-robot planning + execution path. No ros2_control, no
controller_manager, no mock hardware -- kuka_eki_controller_node (run
separately, see README) is the only thing between MoveGroup and the KRC4.

robot_state_publisher is fed by the REAL /joint_states this controller
node publishes, so RViz/TF show the actual robot, not a simulated loopback.

Usage:
    ros2 launch kuka_eki_bridge eki_moveit_planning.launch.py \\
        robot_model:=kr6_r900_2 robot_family:=agilus launch_rviz:=true

    # in a second terminal:
    ros2 run kuka_eki_bridge kuka_eki_controller
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_family = LaunchConfiguration("robot_family")
    moveit_config_pkg = LaunchConfiguration("moveit_config")
    launch_rviz = LaunchConfiguration("launch_rviz")

    rviz_config_file = (
        get_package_share_directory("kuka_resources") + "/config/planning_6_axis.rviz"
    )

    eki_controllers_yaml = (
        get_package_share_directory("kuka_eki_bridge") + "/config/eki_controllers.yaml"
    )

    moveit_config = (
        MoveItConfigsBuilder(f"kuka_{moveit_config_pkg.perform(context)}")
        .robot_description(
            file_path=get_package_share_directory(f"kuka_{robot_family.perform(context)}_support")
            + f"/urdf/{robot_model.perform(context)}.urdf.xacro",
            # mode:=mock keeps the <ros2_control> block in the URDF valid for
            # xacro parsing even though nothing loads a controller_manager
            # against it -- no hardware plugin is ever instantiated here.
            mappings={"mode": "mock"},
        )
        .robot_description_semantic(
            get_package_share_directory(f"kuka_{moveit_config_pkg.perform(context)}_moveit_config")
            + f"/urdf/{robot_model.perform(context)}.srdf"
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path=eki_controllers_yaml)
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .joint_limits(
            file_path=get_package_share_directory(f"kuka_{robot_family.perform(context)}_support")
            + f"/config/{robot_model.perform(context)}_joint_limits.yaml"
        )
        .to_moveit_configs()
    )

    move_group_server = Node(
       package="moveit_ros_move_group",
       executable="move_group",
       output="screen",
       parameters=[
            moveit_config.to_dict(),
            {"publish_planning_scene_hz": 30.0},
            {"trajectory_execution.execution_duration_monitoring": False},
        ],
    )
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": moveit_config.robot_description["robot_description"]}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "error"],
        parameters=[
            {"robot_description_kinematics": {
                "manipulator": {"kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin"}
            }},
        ],
    )

    return [robot_state_publisher, move_group_server, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_model", default_value="kr6_r900_2"),
        DeclareLaunchArgument("robot_family", default_value="agilus"),
        DeclareLaunchArgument("moveit_config", default_value="kr"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
