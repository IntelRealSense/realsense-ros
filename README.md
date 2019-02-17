# ROS Wrapper for Intel&reg; RealSense&trade; Devices
These are packages for using Intel RealSense cameras (D400 series and the SR300) with ROS.

## Installation Instructions

The following instructions support ROS Indigo, on **Ubuntu 14.04**, and ROS Kinetic, on **Ubuntu 16.04**.

#### The simplest way to install on a clean machine is to follow the instructions on the [.travis.yml](https://github.com/intel-ros/realsense/blob/development/.travis.yml) file. It basically summerize the elaborate instructions in the following 3 steps:

### Step 1: Install the latest Intel&reg; RealSense&trade; SDK 2.0
- #### Install from [Debian Package](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages) - In that case treat yourself as a developer. Make sure you follow the instructions to also install librealsense2-dev package.

#### OR
- #### Build from sources by downloading the latest [Intel&reg; RealSense&trade; SDK 2.0](https://github.com/IntelRealSense/librealsense/releases/tag/v2.18.1) and follow the instructions under [Linux Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

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
To start the camera node in ROS:

```bash
roslaunch realsense2_camera rs_camera.launch
```

This will stream all camera sensors and publish on the appropriate ROS topics.

Other stream resolutions and frame rates can optionally be provided as parameters to the 'rs_camera.launch' file.

### Published Topics
The published topics differ according to the device and parameters.
After running the above command with D435i attached, the following list of topics will be available (This is a partial list. For full one type `rostopic list`):
- /camera/color/camera_info
- /camera/color/image_raw
- /camera/depth/camera_info
- /camera/depth/image_rect_raw
- /camera/extrinsics/depth_to_color
- /camera/extrinsics/depth_to_infra1
- /camera/extrinsics/depth_to_infra2
- /camera/infra1/camera_info
- /camera/infra1/image_rect_raw
- /camera/infra2/camera_info
- /camera/infra2/image_rect_raw
- /camera/gyro/imu_info
- /camera/gyro/sample
- /camera/accel/imu_info
- /camera/accel/sample

The "/camera" prefix is the default and can be changed. Check the rs_multiple_devices.launch file for an example.
If using D435 or D415, the gyro and accel topics wont be available. Likewise, other topics will be available when using TM265 (see below).

### Launch parameters
The following parameters are available by the wrapper:
- **serial_no**: will attach to the device with the given serial number. Default, attach to available RealSense device in random.
- **rosbag_filename**: Will publish topics from rosbag file.
- enable_tm2: if set to true Will wait on start until the TM265 device is ready.
- **initial_reset**: On occasions the device was not closed properly and due to firmware issues needs to reset. If set to true, the device will reset prior to usage.
- **align_depth**: If set to true, will publish additional topics with the all the images aligned to the depth image.</br>
The topics are of the form: ```/camera/aligned_depth_to_color/image_raw``` etc.
- **filters**: any of the following options, separated by commas:</br>
 - ```colorizer```: will color the depth image. On the depth topic an RGB image will be published, instead of the 16bit depth values .
 - ```pointcloud```: will add a pointcloud topic `/camera/depth/color/points`. The texture of the pointcloud can be modified in rqt_reconfigure (see below) or using the parameters: `pointcloud_texture_stream` and `pointcloud_texture_index`. Run rqt_reconfigure to see available values for these parameters.
 - The following filters have detailed descriptions in : https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md
   - ```disparity```
   - ```spatial```
   - ```temporal```
   - ```decimation```
- **enable_sync**: gathers closest frames of different sensors, infra red, color and depth, to be sent with the same timetag. This happens automatically when such filters as pointcloud are enabled.
- ***<stream_type>*_width**, ***<stream_type>*_height**, ***<stream_type>*_fps**: <stream_type> can be any of *infra, color, fisheye, depth, gyro, accel, pose*. Sets the required format of the device. If the specified combination of parameters is not available by the device, the stream will not be published. Setting a value to 0, will choose the first format in the inner list. (i.e. consistent between runs but not defined). Note: for gyro accel and pose, only _fps option is meaningful.
- **enable_*<stream_name>***: Choose whether to enable a specified stream or not. Default is true. <stream_name> can be any of *infra1, infra2, color, depth, fisheye, fisheye1, fisheye2, gyro, accel, pose*.
- **tf_prefix**: By default all frame's ids have the same prefix - `camera_`. This allows changing it per camera.
- **base_frame_id**: defines the frame_id all static transformations refers to.
- **spatial_frame_id**: defines the frame_id that `pose` topic refers to.
- **All the rest of the frame_ids can be found in the template launch file: [nodelet.launch.xml](./realsense2_camera/launch/includes/nodelet.launch.xml)**
- **unite_imu_method**: The D435i and TM265 cameras have built in IMU components which produce 2 unrelated streams: *gyro* - which shows angular velocity and *accel* which shows linear acceleration. Each with it's own frequency. By default, 2 corresponding topics are available, each with only the relevant fields of the message sensor_msgs::Imu are filled out.
Setting *unite_imu_method* creates a new topic, *imu*, that replaces the default *gyro* and *accel* topics. Under the new topic, all the fields in the Imu message are filled out.
 - linear_interpolation: Each message contains the last original value of item A interpolated with the previous value of item A, combined with the last original value of item B on last item B's timestamp. (items A and B are accel and gyro but without specific)
 - copy: For each new message, accel or gyro, the relevant fields and timestamp are filled out while the others maintain the previous data.
- **clip_distance**: remove from the depth image all values above a given value (meters). Disable by giving negative value (default)
- **linear_accel_cov**, **angular_velocity_cov**: sets the variance given to the Imu readings. For the TM265, these values are being modified by the inner confidence value.
- **hold_back_imu_for_frames**: Images processing takes time. Therefor there is a time gap between the moment the image arrives at the wrapper and the moment the image is published to the ROS environment. During this time, Imu messages keep on arriving and a situation is created where an image with earlier timestamp is published after Imu message with later timestamp. If that is a problem, setting *hold_back_imu_for_frames* to *true* will hold the Imu messages back while processing the images and then publish them all in a burst, thus keeping the order of publication as the order of arrival. Note that in either case, the timestamp in each message's header reflects the time of it's origin.

### RGBD Point Cloud
Here is an example of how to start the camera node and make it publish the RGBD point cloud using aligned depth topic.
```bash
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
```
Then open rviz to watch the pointcloud:
<p align="center"><img src="https://user-images.githubusercontent.com/17433152/35396613-ddcb1d6c-01f5-11e8-8887-4debf178d0cc.gif" /></p>

### Aligned Depth Frames
Here is an example of how to start the camera node and make it publish the aligned depth stream to other available streams such as color or infra-red.
```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```
<p align="center"><img width=50% src="https://user-images.githubusercontent.com/17433152/35343104-6eede0f0-0132-11e8-8866-e6c7524dd079.png" /></p>

### Set Camera Controls Using Dynamic Reconfigure Params
The following command allow to change camera control values using [http://wiki.ros.org/rqt_reconfigure].
```bash
rosrun rqt_reconfigure rqt_reconfigure
```
<p align="center"><img src="https://user-images.githubusercontent.com/40540281/52912434-30783180-32ba-11e9-8ab8-b08be5eba434.PNG" /></p>

### Work with multiple cameras
Here is an example of how to start the camera node and streaming with two cameras using the [rs_multiple_devices.launch](./realsense2_camera/launch/rs_multiple_devices.launch).
```bash
roslaunch realsense2_camera rs_multiple_devices.launch serial_no_camera1:=<serial number of the first camera> serial_no_camera2:=<serial number of the second camera>
```
The camera serial number should be provided to `serial_no_camera1` and `serial_no_camera2` parameters. One way to get the serial number is from the [rs-enumerate-devices](https://github.com/IntelRealSense/librealsense/blob/58d99783cc2781b1026eeed959aa3f7b562b20ca/tools/enumerate-devices/readme.md) tool.
```bash
rs-enumerate-devices | grep Serial
```

Another way of obtaining the serial number is connecting the camera alone, running
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

## Using TM265 ##
When using TM265 Tracking Module you should specify it with a parameter `enable_tm2` as is set in the following launch file:
```bash
roslaunch realsense2_camera rs_tm260.launch
```
The TM265 sets its usb unique ID during initialization and without this parameter it wont be found.
Once running it will publish, among others, the following topics:
- /camera/odom/sample
- /camera/accel/sample
- /camera/gyro/sample
- /camera/fisheye1/image_raw
- /camera/fisheye2/image_raw


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
