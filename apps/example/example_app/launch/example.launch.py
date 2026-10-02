from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='example_app',
            executable='heartbeat',
            output='screen',
        ),
    ])
