#!/bin/bash

# Installation script for realsense_navigation
# Requires root privileges

sudo cp ./robot_description/kobuki_minimal_r200.urdf.xacro /opt/ros/$ROS_DISTRO/share/turtlebot_description/robots/
sudo cp ./robot_description/r200.launch.xml /opt/ros/$ROS_DISTRO/share/turtlebot_bringup/launch/includes/3dsensor/

sudo mkdir -p /opt/ros/$ROS_DISTRO/share/realsense_navigation
sudo cp -r ./* /opt/ros/$ROS_DISTRO/share/realsense_navigation
