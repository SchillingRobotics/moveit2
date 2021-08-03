import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils.moveit_configs_builder import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_name="panda.urdf.xacro")
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        .moveit_configs()
    )

    # MoveGroupInterface demo executable
    run_move_group_demo = Node(
        name="run_move_group",
        package="run_move_group",
        executable="run_move_group",
        prefix="xterm -e",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([run_move_group_demo])
