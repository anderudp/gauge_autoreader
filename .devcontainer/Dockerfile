ARG ROS_DISTRO=jazzy
FROM ros:$ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /root/ws

RUN if id -u 1000; then userdel `id -un 1000` ; fi

RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y \
    git \
    build-essential \
    openssh-client \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-serial \
    python3-yaml \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-dynamixel-sdk \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-graph \
    ros-$ROS_DISTRO-librealsense2*

RUN pip install --break-system-packages \
    lifxlan \
    xarm-python-sdk

ENV SHELL /bin/bash
