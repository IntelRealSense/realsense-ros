# Intel® RealSense™ SDK for Linux ROS Samples

## Features
These samples illustrate how to develop OSRF&reg; ROS* applications using the Intel® RealSense™ [ZR300](http://click.intel.com/intelr-realsensetm-development-kit-featuring-the-zr300.html) camera for Object Library (OR, Person Library (PT), and Simultaneous Localization And Mapping (SLAM).

## Installation Instructions

The Intel RealSense SDK for Linux is used as the base for these ROS node.  Full installation information for this SDK is available at https://software.intel.com/sites/products/realsense/intro. Here is the quick guide:

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

## Usage Instructions
- [Camera](realsense_ros_camera/README.md): This ROS node implements use of ZR300 camera as a standard ROS camera node.
- [Object Recognition, Localization, and Tracking](realsense_ros_object/README.md): This ROS node demonstrates use of ZR300 camera above to implement Object Recognition, Localization, and Tracking functionality.
- [Person Tracking and Analysis](realsense_ros_person/README.md): This ROS node demonstrates use of the ZR300 camera above to implement Person Detection, Tracking, and Gesture analysis.
- [SLAM](realsense_ros_slam/README.md): This ROS node demonstrates use of ZR300 camera above for simultaneous location and mapping (SLAM), relocalization, and occupancy map generation.

## Record and Playback Support
These sample ROS nodes supporting using the ROS 'rosbag' tool for recording and playback.

For recording to a specific file.  If you omit the bag_path argument, it will default to $HOME/test.bag
```bash
$ roslaunch realsense_ros_camera record_bag.launch bag_path:=<path to file>
```
When you ready to conclude the recording, press Ctrl-C and the recording will end gracefully.

For playing back from a specific file.  Like above, if you omit the bag_path argument, it will default to $HOME/test.bag
```bash
$ roslaunch realsense_ros_camera play_bag.launch bag_path:=<path to file>
```

All of the sample ROS nodes above accept an optional bag_path parameter to allow usage and testing from a pre-recorded file instead of the live camera streams.

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
