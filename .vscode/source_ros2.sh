#!/usr/bin/env bash
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "${ROS_WORKSPACE}/install/setup.bash" ]; then
    source "${ROS_WORKSPACE}/install/setup.bash"
fi