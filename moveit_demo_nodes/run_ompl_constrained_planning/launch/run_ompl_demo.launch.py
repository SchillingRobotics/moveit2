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
        .to_moveit_configs()
    )

    # MoveGroupInterface demo executable
    run_move_group_demo = Node(
        name="run_ompl_constrained_planning",
        package="run_ompl_constrained_planning",
        executable="run_ompl_constrained_planning",
        output="screen",
        #    prefix='kitty -e gdb -e run --args',
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([run_move_group_demo])
