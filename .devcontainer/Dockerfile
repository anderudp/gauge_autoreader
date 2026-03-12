FROM ros:jazzy
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /root/ws

RUN if id -u 1000; then userdel `id -un 1000` ; fi

RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y \
    git \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-serial \
    python3-yaml \
    ros-jazzy-rviz2 \
    ros-jazzy-dynamixel-sdk

ENV SHELL /bin/bash
