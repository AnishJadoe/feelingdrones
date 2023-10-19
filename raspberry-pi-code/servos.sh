#!/bin/bash

# Navigate to the right directory and source it
cd /home/user/ros/FeelingDroneRPi
source /opt/ros/${ROS_DISTRO}/setup.sh
source ws/install/setup.bash

ros2 run sts3032_motor_driver actuator