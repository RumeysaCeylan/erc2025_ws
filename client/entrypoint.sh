#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash

# IPv6 format düzeltme
if [[ ! -z "$ROS_DISCOVERY_SERVER_RAW" ]]; then
    export ROS_DISCOVERY_SERVER="[$ROS_DISCOVERY_SERVER_RAW]:11811"
fi

echo "Running as role: $CHATTER_ROLE"

exec ros2 run teleop_twist_keyboard teleop_twist_keyboard
