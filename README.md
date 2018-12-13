# ROS Wrapper for Intel&reg; RealSense&trade; Devices
These are packages for using Intel RealSense cameras (D400 series and the SR300) with ROS.

## Installation Instructions

The following instructions support ROS Indigo, on **Ubuntu 14.04**, and ROS Kinetic, on **Ubuntu 16.04**.

#### The simplest way to install on a clean machine is to follow the instructions on the [.travis.yml](https://github.com/intel-ros/realsense/blob/development/.travis.yml) file. It basically summerize the elaborate instructions in the following 3 steps:

### Step 1: Install the latest Intel&reg; RealSense&trade; SDK 2.0
- #### Install from [Debian Package](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages) - In that case treat yourself as a developer. Make sure you follow the instructions to also install librealsense2-dev package.

#### OR
- #### Build from sources by downloading the latest [Intel&reg; RealSense&trade; SDK 2.0](https://github.com/IntelRealSense/librealsense/releases/tag/v2.17.0) and follow the instructions under [Linux Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

### Step 2: Install the ROS distribution
- #### Install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu), on Ubuntu 16.04

### Step 3: Install Intel&reg; RealSense&trade; ROS from Sources
- Create a [catkin](http://wiki.ros.org/catkin#Installing_catkin) workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
```
- Clone the latest Intel&reg; RealSense&trade; ROS from [here](https://github.com/intel-ros/realsense/releases) into 'catkin_ws/src/'

```bash
catkin_init_workspace 
cd ..
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Usage Instructions

### Start the camera node
To start the camera node in ROS, install rgbd_launch:

```bash
sudo apt-get install ros-kinetic-rgbd-launch
```
Then type:

```bash
roslaunch realsense2_camera rs_camera.launch
```

This will stream all camera sensors and publish on the appropriate ROS topics.

Other stream resolutions and frame rates can optionally be provided as parameters to the 'rs_camera.launch' file.

### RGBD Point Cloud
Here is an example of how to start the camera node and make it publish the RGBD point cloud using aligned depth topic.
```bash
roslaunch realsense2_camera rs_rgbd.launch
```
<p align="center"><img src="https://user-images.githubusercontent.com/17433152/35396613-ddcb1d6c-01f5-11e8-8887-4debf178d0cc.gif" /></p>

### Aligned Depth Frames
Here is an example of how to start the camera node and make it publish the aligned depth stream to other available streams such as color or infra-red.
```bash
roslaunch realsense2_camera rs_aligned_depth.launch
```
<p align="center"><img width=50% src="https://user-images.githubusercontent.com/17433152/35343104-6eede0f0-0132-11e8-8866-e6c7524dd079.png" /></p>

### Set Camera Controls Using Dynamic Reconfigure Params
The following command allow to change camera control values using [http://wiki.ros.org/rqt_reconfigure].
```bash
rosrun rqt_reconfigure rqt_reconfigure
```
<p align="center"><img src="https://user-images.githubusercontent.com/17433152/35397261-b4e846ac-01f7-11e8-8512-1e3671b4003b.png" /></p>

### Work with multiple cameras
Here is an example of how to start the camera node and streaming with two cameras using the [rs_multiple_devices.launch](./realsense2_camera/launch/rs_multiple_devices.launch).
```bash
roslaunch realsense2_camera rs_multiple_devices.launch serial_no_camera1:=<serial number of the first camera> serial_no_camera2:=<serial number of the second camera>
```
The camera serial number should be provided to `serial_no_camera1` and `serial_no_camera2` parameters. One way to get the serial number is from the [rs-enumerate-devices](https://github.com/IntelRealSense/librealsense/blob/58d99783cc2781b1026eeed959aa3f7b562b20ca/tools/enumerate-devices/readme.md) tool.

Another war of obtaining the serial number is connecting the camera alone, running 
```bash
roslaunch realsense2_camera rs_camera.launch
```
and looking for the serial number in the log printed to screen under "[INFO][...]Device Serial No:".

Another way to use multiple cameras is running each from a different terminal. Make sure you set a different namespace for each camera using the "camera" argument:

```bash
roslaunch realsense2_camera rs_camera.launch camera:=cam_1 serial_no:=<serial number of the first camera>
roslaunch realsense2_camera rs_camera.launch camera:=cam_2 serial_no:=<serial number of the second camera>
...

```
### Enabling post processing filters.
realsense2_camera includes some built in post processing filters:<br>
colorizer - creates an RGB image instead of depth image. Used to visualize the depth image.<br>
spatial - filter the depth image spatially.<br>
temporal - filter the depth image temporally.<br>
pointcloud - it is now possible to enable point cloud with the same command as any other post processing filter.<br>

The colorizer filter replaces the image in the topic: /<camera>/depth/image_rect_raw.
The spatial and temporal filters affect the depth image and all that is derived from it, i.e. pointcloud and colorizer.

to activate the filters, use the argument "filters" and deperate them with a comma:
```bash
roslaunch realsense2_camera rs_camera.launch filters:=temporal,spatial,pointcloud
```


## Packages using RealSense ROS Camera
| Title | Links |
| ----- | ----- |
| ROS Object Analytics | [github](https://github.com/intel/ros_object_analytics) / [ROS Wiki](http://wiki.ros.org/IntelROSProject)

## Known Issues
* This ROS node does not currently support [ROS Lunar Loggerhead](http://wiki.ros.org/lunar).
* This ROS node does not currently work with [ROS 2](https://github.com/ros2/ros2/wiki).
* This ROS node currently does not provide the unit-tests which ensure the proper operation of the camera.  Future versions of the node will provide ROS compatible unit-tests.

## License
Copyright 2018 Intel Corporation

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
