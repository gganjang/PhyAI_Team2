from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('http_port', default_value='23517',
                              description='Browser view port; pick another if a teammate on the same dev PC uses it'),
        Node(
            package='piper_mujoco_sim',
            executable='sim',
            parameters=[{'http_port': LaunchConfiguration('http_port')}],
            output='screen',
        ),
    ])
