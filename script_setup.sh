#!/bin/bash
#
# This script sets up the Sheldon robot software stack, from a fresh install of Ubuntu 16.04
#
# Assumptions:
# 1) You are a member of the github.com/mattcurfman repo's below, and you've setup your github account with your
# SSH public key to be granted access.
# 2) You are on a public internet, or have manually setup your proxies for corporate network access
#


# Run script as root, unless permissions are dropped elsewhere.  This allows root password to be provided once at start of script
export _DEFAULT_USER=$USER
sudo -E bash <<"EOF"

export NPROCS=`grep -c ^processor /proc/cpuinfo`


function install_ros {
if [ -f "/etc/apt/sources.list.d/ros-latest.list" ] 
then
  echo "ROS is already installed, skipping this step"
else
  apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
  sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  apt-get update
  apt-get -y install ros-kinetic-desktop-full
  rosdep init

sudo -E -u $_DEFAULT_USER bash <<"EOF2"
  rosdep update
  echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
EOF2
fi
}


function install_realsense {
if [ -f "/etc/apt/sources.list.d/realsense-latest.list" ] 
then
  echo "RealSense is already installed, skipping this step"
else
  apt-key adv --keyserver keys.gnupg.net --recv-key D6FB2970 
  sh -c 'echo "deb http://realsense-alm-public.s3.amazonaws.com/apt-repo xenial main" > /etc/apt/sources.list.d/realsense-latest.list'
fi

apt update 
apt install -y librealsense-object-recognition-dev librealsense-persontracking-dev librealsense-slam-dev librealsense-utils 
}

function install_robot_common {
# Perform the following with normal user permissions (e.g. drop root)
sudo -E -u $_DEFAULT_USER bash <<"EOF2"
  source /opt/ros/kinetic/setup.bash
  cd
  mkdir -p catkin_ws/src
  cd catkin_ws/src/
  catkin_init_workspace 
  git clone http://github.intel.com/IntelRealSense/realsense_ros
  cd 
  cd catkin_ws
#  catkin_make
EOF2
}

install_ros
install_realsense
install_robot_common

ldconfig

EOF

