#!/usr/bin/env bash

rm -rf /ws
mkdir /ws
cp -r /workspace/{src,launch} /ws

source /opt/ros/jazzy/setup.bash
cd /ws && colcon build
source ./install/setup.bash && ros2 launch ./launch/launch.py

