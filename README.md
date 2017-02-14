# Intel® RealSense™ SDK for Linux ROS Samples

## Features
These samples illustrate how to develop OSRF&reg; ROS* applications using Intel® RealSense™ cameras for Object Library (OR), Person Library (PT), and Simultaneous Localization And Mapping (SLAM).

## Installation Instructions

The Intel RealSense SDK for Linux is used as the base for these ROS node.  Public installation information for RealSense for Ubuntu 16.04 is available at https://software.intel.com/sites/products/realsense/intro/

```bash
# Install ROS Kinetic full desktop environment
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-get update
apt-get -y install ros-kinetic-desktop-full
rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

# Install Intel RealSense SDK for Linux
apt-key adv --keyserver keys.gnupg.net --recv-key D6FB2970 
sh -c 'echo "deb http://realsense-alm-public.s3.amazonaws.com/apt-repo xenial main" > /etc/apt/sources.list.d/realsense-latest.list'
apt update 
apt install -y librealsense-object-recognition-dev librealsense-persontracking-dev librealsense-slam-dev libopencv-dev uvcvideo-realsense-dkms

# Download and compile ROS wrappers for Intel RealSense SDK for Linux
mkdir -p catkin_ws/src
cd catkin_ws/src/
catkin_init_workspace 
git clone git@github.intel.com:IntelRealSense/realsense_ros.git
```

## Using ROS Wrappers
- [Camera](realsense_ros_camera/README.md): This ROS node (fill in content).
- [Tracking](realsense_ros_object/README.md): This ROS node (fill in content).
- [Person](realsense_ros_person/README.md): This ROS node (fill in content).
- [SLAM](realsense_ros_slam/README.md): This ROS node (fill in content).

## License
Copyright 2017 Intel Corporation

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this project except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

**Other names and brands may be claimed as the property of others*
