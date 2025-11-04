#!/usr/bin/env bash
if [ -f ~/.bashrc ]; then
  source ~/.bashrc
fi

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi

if [ -f "${PWD}/install/setup.bash" ]; then
  source "${PWD}/install/setup.bash"
fi

echo "[ROS2] Environment sourced from: ${PWD}"