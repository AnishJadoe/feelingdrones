from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sts3032_motor_driver',
            executable='actuator',
            name='sts30302_servo'
        ),
        Node(
            package='mpr121_pubsub',
            executable='talker',
            name='touch_sensor'
        ),
    ])