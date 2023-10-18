from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_broadcasters',
            executable='tf2_broadcaster',
            name='broadcaster1',
        ),
    ])