import sys
import os
import launch

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch import LaunchService
from launch.actions.execute_process import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration
from pathlib import Path


def generate_launch_description():
    controllers_path = os.path.join(get_package_share_directory('moveit_resources'), 'mara_moveit_config', 'config', 'controllers.yaml')
    controllers = LaunchConfiguration('controllers',
                                       default=controllers_path)
    assert Path(controllers_path).is_file()
    ld = LaunchDescription([
        Node(package='moveit_ros_move_group', 
             node_executable='move_group', 
             output='screen', 
             #parameters=[controllers_path, {'enabled_param1': 'true'}])
             parameters=[{'x': -0.6},
                        {'y': 0.4},
                        {'z': 0.1},
                        {'camera_frame': 'camera_frame'}])
    ])
    return ld
