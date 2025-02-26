from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('panda_can_rcv'),
        'config',
        'panda_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='panda_can_rcv',
            executable='can_listener_node',
            name='can_listener_node',
            output='screen',
            parameters=[config_file_path],
        ),
    ])