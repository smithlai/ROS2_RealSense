from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense',
            executable='simple_publisher',
            output='screen'),

        Node(
            package='realsense',
            executable='simple_subscriber',
            output='screen'),
    ])