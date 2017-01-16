#!/bin/bash
. /opt/ros/$ROS_DISTRO/share/rosbash/rosbash

roscd $1

base_folder=$(basename `pwd`)

absolute_path=`pwd`

