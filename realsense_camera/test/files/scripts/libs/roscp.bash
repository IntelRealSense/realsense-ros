#!/bin/bash
. /opt/ros/$ROS_DISTRO/share/rosbash/rosbash

if [ $# != 2 ]; then
echo "lack of source file or target dir..."
exit 1
fi

roscp realsense_camera $1 $2
