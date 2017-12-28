# ROS Wrapper for Intel&reg; RealSense&trade; Devices
These are packages for using Intel RealSense cameras (D400 series and the SR300) with ROS.

## Installation Instructions

The following instructions support ROS Indigo, on **Ubuntu 14.04**, and ROS Kinetic, on **Ubutnu 16.04**.

### Step 1: Install the latest Intel&reg; RealSense&trade; SDK 2.0
Download the latest [Intel&reg; RealSense&trade; SDK 2.0](https://github.com/IntelRealSense/librealsense/releases/latest) and follow the instructions under [Linux Installation](https://github.com/IntelRealSense/librealsense/blob/development/doc/installation.md).

### Step 2: Install the ROS distribution
- #### Install [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu), on Ubuntu 14.04

#### OR
- #### Install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu), on Ubuntu 16.04

### Step 3: Install Intel&reg; RealSense&trade; ROS from Sources
- Create a [catkin](http://wiki.ros.org/catkin#Installing_catkin) workspace
```bash
mkdir -p ~/catkin_ws/src
cd catkin_ws/src/
```
- Clone the latest Intel&reg; RealSense&trade; ROS from [here](https://github.com/intel-ros/realsense/releases) into 'catkin_ws/src/'

```bash
catkin_init_workspace 
cd ..
catkin_make clean
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make install
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Usage Instructions

### Start the camera node
To start the camera node in ROS, plug in the camera, then type the following command:

```bash
roslaunch realsense_ros_camera rs_camera.launch
```

This will stream all camera sensors and publish on the appropriate ROS topics.

Other stream resolutions and frame rates can optionally be provided as parameters to the 'rs_camera.launch' file.

### Visualize Depth Point Cloud

To start the camera node in ROS and view the depth pointcloud, type the following command:

```bash
roslaunch realsense_ros_camera demo_pointcloud.launch
```

This will launch [RViz](http://wiki.ros.org/rviz) and display the depth pointcloud.


## Known Issues
* This ROS node does not currently provide any dynamic reconfigure support for camera properties/presets.
* This ROS node does not currently support [ROS Lunar Loggerhead](http://wiki.ros.org/lunar).
* This ROS node does not currently work with [ROS 2](https://github.com/ros2/ros2/wiki).
* This ROS node currently does not provide the unit-tests which ensure the proper operation of the camera.  Future versions of the node will provide ROS compatible unit-tests.

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
