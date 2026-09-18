#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/chapt7/chapt7_ws/install/setup.bash
source ~/chapt10/chapt10_ws/install/setup.bash
exec ros2 launch chapt10_bringup navigation.launch.py "$@"
