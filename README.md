# Intel® RealSense™ SDK for Linux ROS Samples

## Features
These samples illustrate how to develop OSRF&reg; ROS* applications using the Intel® RealSense™ [ZR300](http://click.intel.com/intelr-realsensetm-development-kit-featuring-the-zr300.html) camera for Object Library (OR), Person Library (PT), and Simultaneous Localization And Mapping (SLAM).

## Installation Instructions

The Intel RealSense SDK for Linux is used as the base for these ROS node.  Full installation information for the SDK is available at https://software.intel.com/sites/products/realsense/intro. Here is the quick setup guide:

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
sudo apt install -y librealsense-object-recognition-dev librealsense-persontracking-dev librealsense-slam-dev libopencv-dev

# Download and compile ROS wrappers for Intel RealSense SDK for Linux
mkdir -p catkin_ws/src
cd catkin_ws/src/
catkin_init_workspace 
git clone https://github.com/IntelRealSense/realsense_samples_ros
cd ..
catkin_make
source devel/setup.bash
```

## Usage Instructions
- [Camera](realsense_ros_camera/README.md): This ROS node implements use of ZR300 camera as a standard ROS camera node.
- [Object Recognition, Localization, and Tracking](realsense_ros_object/README.md): This ROS node demonstrates use of ZR300 camera above to implement Object Recognition, Localization, and Tracking functionality.
- [Person Tracking and Analysis](realsense_ros_person/README.md): This ROS node demonstrates use of the ZR300 camera above to implement Person Detection, Tracking, and Gesture analysis.
- [SLAM](realsense_ros_slam/README.md): This ROS node demonstrates use of ZR300 camera above for simultaneous location and mapping (SLAM), relocalization, and occupancy map generation.

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
