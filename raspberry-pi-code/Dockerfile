# Build arguments
ARG ROS_DISTRO=humble

# Install python packages

# Base image
FROM ros:${ROS_DISTRO}-ros-base

# An ARG declared before a FROM is outside of a build stage, so it can’t be used in any instruction after a FROM

# Install additional ros packages and other libraries
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get upgrade -yq && apt-get install -y \
    python3.10\
    python3-pip\
    python3-dev\
    python3-rpi.gpio\
    git \
    make \
    vim \
    nano \ 
    && rm -rf /var/lib/apt/lists/*

# Install necessary python packages
RUN pip3 install adafruit-circuitpython-mpr121
RUN sudo pip3 install adafruit-blinka

# Add copy of local workspace and setup script
WORKDIR /home/user/ros/FeelingDroneRPi
ADD ws/ ./ws/
WORKDIR /home/user/ros/FeelingDroneRPi/ws

# Setup workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select mpr121_pubsub

# Add the entrypoint script
ADD entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]