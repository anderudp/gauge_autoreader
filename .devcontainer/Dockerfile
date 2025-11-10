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
    python3-pynput \
    python3-serial \
    ros-jazzy-rviz2

ENV SHELL /bin/bash
