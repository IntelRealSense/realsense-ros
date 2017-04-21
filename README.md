# Intel® RealSense™ SDK for Linux ROS Samples, for DS5

## Features
These samples illustrate how to develop OSRF&reg; ROS* applications using the Intel® RealSense™ "DS5" R10 camera. 

*Note:* This is pre-release support for DS5, without middleware support 

## Installation Instructions

libRealsense version 2 is required to successfully build the DS5 support in this camera, version v2.5.2.  Please see Alex Sherman for debian packages for this release, which are not available publically.

```bash
# Install ROS Kinetic full desktop environment
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get update
sudo apt-get -y install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install Intel RealSense SDK for Linux
sudo apt-key adv --keyserver keys.gnupg.net --recv-key D6FB2970 
sudo sh -c 'echo "deb http://realsense-alm-public.s3.amazonaws.com/apt-repo xenial main" > /etc/apt/sources.list.d/realsense-latest.list'
sudo apt update 
sudo apt install -y librealsense-dev
# Then install librealsense2 packages from Alex Sherman at this point in instructions, using installation instructions provided by his team

# Download and compile ROS wrappers for Intel RealSense SDK for Linux
mkdir -p catkin_ws/src
cd catkin_ws/src/
catkin_init_workspace 
git clone -b ds5 https://github.com/IntelRealSense/realsense_samples_ros
cd ..
catkin_make
source devel/setup.bash
```

## Usage Instructions
- [Camera](realsense_ros_camera/README.md): This ROS node implements use of R200, LR200, ZR300, and LR410 camera as a standard ROS camera node.

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
