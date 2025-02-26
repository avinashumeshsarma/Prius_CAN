from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='csv_publisher',
            executable='csv_publisher_node',
            name='csv_publisher_node',
            output='screen',
        )
    ])
