#!/bin/bash
#

# Run script as root, unless permissions are dropped elsewhere.  This allows root password to be provided once at start of script
export _DEFAULT_USER=$USER
sudo -E bash <<"EOF"

export NPROCS=`grep -c ^processor /proc/cpuinfo`


function init_install {
  while fuser /var/lib/dpkg/lock >/dev/null 2>&1 ; do
    echo "Waiting for other software managers to stop ..."
    killall aptd
    killall apt-get
    killall apt
    sleep 0.5
  done 
}

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
  apt update 
  apt install -y librealsense-object-recognition-dev librealsense-persontracking-dev librealsense-slam-dev librealsense-utils 
fi
}

function install_robot_common {
# Perform the following with normal user permissions (e.g. drop root)
sudo -E -u $_DEFAULT_USER bash <<"EOF2"
  source /opt/ros/kinetic/setup.bash
  cd
  if [ -d "catkin_ws/src/realsense_ros" ] 
  then
    echo "Repo realsense_ros is already presenting, skipping this step"
  else
    echo "Creating new realsense_ros repository"
    mkdir -p catkin_ws/src
    cd catkin_ws/src/
    catkin_init_workspace 
    git clone http://github.intel.com/IntelRealSense/realsense_ros
  fi
EOF2
}

init_install
install_ros
install_realsense
install_robot_common

ldconfig

EOF

