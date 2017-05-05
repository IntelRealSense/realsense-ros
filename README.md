# ROS* support for Intel® RealSense™ R410 camera 

Version 2.5.3

## Features
This package illustrates how to develop OSRF&reg; ROS* applications using the Intel® RealSense™ R410 camera. 

**Important:** This is pre-release software, please read the known issues below

**Note:** This pre-release software requires Intel RealSense R410 camera FW 5.6.4.0 or newer. A warning message will be displayed if the software finds camera FW that is older.

## Installation Instructions

The following instructions support ROS Indigo, on Ubuntu 14.04, and ROS Kinetic, on Ubutnu 16.04.  Please follow the instructions for Step 1 for the OS you are using, then proceed to Step 2

### Step 1: Install ROS Indigo, on Ubuntu 14.04

The following steps will install the ROS Indigo software framework, on a clean Ubuntu 14.04 machine:
```bash
# Install ROS Indigo full desktop environment
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get update
sudo apt-get -y install ros-indigo-desktop-full ros-indigo-depthimage-to-laserscan
sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 1: Install ROS Kinetic, on Ubuntu 16.04
The following steps will install the ROS Kinetic software framework, on a clean Ubuntu 16.04 machine:
```bash
# Install ROS Kinetic full desktop environment
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get update
sudo apt-get -y install ros-kinetic-desktop-full ros-kinetic-depthimage-to-laserscan
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Install Intel RealSense Support for ROS
After Step 1 above is completed, the additional steps below will compile support for the Intel RealSense R410 camera, from source.  

**Note:** <path_to_realsense_ros_r410.tar.bz2> refers to the archive you received from your customer representative or through the VIP portal.  Please substitute <path_to_realsense_ros_r410.tar.bz2> with the actual archive filename you received.
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src/
catkin_init_workspace 
tar xv <path_to_realsense_ros_r410.tar.bz2>
cd ..
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Finally, to enable camera access as non-root users, the following steps will install the required udev rules.  Note: the steps below will request that your device reboot.
```bash
sudo cp `rospack find librealsense2`/config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo reboot
```

## Usage Instructions

### Default depth stream
To start the camera node in ROS at the default 640x480x30 stream, first plug in the camera, then type the following command:

```bash
roslaunch realsense_ros_camera demo.launch
```

This will display a default ROS viewer with the depth stream from the camera.  To better visualize difference ranges of the depth camera, use the viewer window's range tool in the upper right hand corner dialog.  The default in the viewer is 10.00m.

Additional depth stream resolutions can optionally be provided as parameters to the camera.launch file

### Point Cloud and Laser scan 

To start the camera node in ROS and view the associated pointcloud and simulated laser scan, type the following command:

```bash
roslaunch realsense_ros_camera demo_pointcloud_and_laser.launch
```

This will open RViz and display the camera pointcloud and laser scan

## Additional Non-ROS RealSense standlone utilities
Although not built as part of this ROS package, the provided librealsense2 library contains additional standalone utilities for Linux and the Intel RealSense camera, including viewers with full control for all camera parameteters.  Please see the installation guide in librealsense2\README.md for details on compiling and using these utilities.

## Known Issues
* This pre-release software package requires Intel RealSense R410 camera firmware 5.6.4.0 or newer.  Although it is not built by the ROS instructions above, the librealsense2 package can be built on the commandline using CMake with the parameter "-DBUILD_EXAMPLES-on", which will build a utility called 'cpp-enumerate'.  This utility will print the version number of your camera
* This ROS node currently only supports the Intel RealSense R410 camera model
* This ROS node does not currently provide URDF models for the camera.
* This ROS node does not currently provide any dynamic reconfigure support for camera properties.
* This ROS node does not currently provide support for using the RGBD package, but does support generation of a built-in point cloud topic stream.
* This ROS node currently only provides the Depth stream from the camera sensor.  Future versions of the node will provide access to all camera streams.
* This ROS node currently does not provide the unit-tests which ensure the proper operation of the camera.  Future versions of the node will provide ROS compatible unit-tests.
* The SIGINT handler in librealsense2 2.5.3 is currently enabled incorrectly, which prevents clean shutdown of the ROS camera node.  This will be resolved in a future release

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
