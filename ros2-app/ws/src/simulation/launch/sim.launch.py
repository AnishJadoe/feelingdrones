"""Launch a Gazebo simulation spawning a PX4 drone communicating over ROS2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Gazebo with a drone running PX4 communicating over ROS 2."""
    HOME = os.environ.get('HOME')
    PX4_RUN_DIR = HOME + '/dev/working/PX4-Autopilot'

    os.makedirs(PX4_RUN_DIR, exist_ok=True)
    
    # # GZ - ROS Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (GZ -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Base Pose Ground Truth (GZ->ROS2)
            '/world/cyberzoo/model/x500/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
        ],
        output='screen'
    )

    return LaunchDescription([
        # Set required environment variables
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',
                        HOME + '/dev/working/feeling_drones/ros2-app/build/simulation'),
        SetEnvironmentVariable('PX4_GZ_MODEL',
                               'x500'),
        SetEnvironmentVariable('PX4_GZ_WORLD',
                               'cyberzoo'),
        SetEnvironmentVariable('PX4_GZ_MODEL_POSE',
                               '0,0,0.75,0,0,1.5707963267948966'),
        
        # Launch MicroXRCEAgent to communicate between PX4 and ROS2
        ExecuteProcess(
            cmd=[
                'micro-xrce-dds-agent', 'udp4', '-p', '8888'
            ],
            prefix="bash -c 'sleep 5s; $0 $@'",
            output='screen'),
        # Launch PX4 GZ Sim
        ExecuteProcess(
            cmd=[
                PX4_RUN_DIR + '/build/px4_sitl_default/bin/px4',
                ]),

        # # Launch GZ-ROS2 Bridge
        bridge,
    ])