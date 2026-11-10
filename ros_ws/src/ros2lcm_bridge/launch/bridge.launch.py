import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('ros2lcm_bridge'), 'config', 'bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('lcm_url', default_value=os.environ.get('LCM_LOCOMOTION_URL', '')),
        Node(package='ros2lcm_bridge', executable='bridge_node', output='screen',
             parameters=[LaunchConfiguration('config'),
                         {'lcm_url': ParameterValue(LaunchConfiguration('lcm_url'), value_type=str)}]),
    ])
