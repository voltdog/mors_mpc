from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

config = os.path.join(
      get_package_share_directory('mors_sim'),
      'config',
      'pybullet_config.yaml'
      )
print(config)

def generate_launch_description():

    sim_node = Node(
            package='mors_sim',
            executable='mors_sim',
            name='mors_sim',
            output='screen',
            parameters=[config]
        )

    ld = LaunchDescription()
    ld.add_action(sim_node)
    # ld.add_action(opfunc)

    return ld
