from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        .moveit_configs()
    )

    # MoveGroupInterface demo executable
    run_move_group_demo = Node(
        name="run_ompl_constrained_planning",
        package="run_ompl_constrained_planning",
        executable="run_ompl_constrained_planning",
        output="screen",
        #    prefix='kitty -e gdb -e run --args',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([run_move_group_demo])
