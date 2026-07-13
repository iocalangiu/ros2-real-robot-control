from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follower',
            executable='wall_finder_executable',
            output='screen',
            emulate_tty=True)
    ])