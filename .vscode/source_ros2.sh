#!/usr/bin/env bash
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

if [ -f "${ROS_WORKSPACE}/install/setup.bash" ]; then
    source "${ROS_WORKSPACE}/install/setup.bash"
fi