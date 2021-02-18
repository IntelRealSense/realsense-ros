# ROS2 Wrapper for Intel&reg; RealSense&trade; Devices
These are packages for using Intel RealSense cameras (D400 and L500 series, SR300 camera and T265 Tracking Module) with ROS2.

LibRealSense supported version: v2.42.0 (see [realsense2_camera release notes](https://github.com/IntelRealSense/realsense-ros/releases))

## Installation Instructions
This version supports ROS2 foxy on Ubuntu 20.04.

   ### Step 1: Install the ROS2 distribution
   - #### Install [ROS2 foxy](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/), on Ubuntu 20.04.

   ```bash
  ROS_DISTRO=foxy
  sudo apt update && sudo apt install curl gnupg2 lsb-release
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
  sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
  sudo apt update
  sudo apt install ros-$ROS_DISTRO-ros-base

  #Environment setup
  echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
  source ~/.bashrc

  sudo apt install python3-colcon-common-extensions -y

  #Install argcomplete (optional)
  sudo apt install python3-argcomplete
   ```


  ### Step 2: Install librealsense2:
   ### Install the latest Intel&reg; RealSense&trade; SDK 2.0
   - #### Install from [Debian Package](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages) - In that case treat yourself as a developer. Make sure you follow the instructions to also install librealsense2-dev and librealsense-dkms packages.

   #### OR
   - #### Build from sources by downloading the latest [Intel&reg; RealSense&trade; SDK 2.0](https://github.com/IntelRealSense/librealsense/releases/tag/v2.42.0) and follow the instructions under [Linux Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)


   ### Step 3: Install Intel&reg; RealSense&trade; ROS2 wrapper from Sources
   - Create a ROS2 workspace
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src/
   ```
   - Clone the latest Foxy Intel&reg; RealSense&trade;  wrapper from [here](https://github.com/IntelRealSense/realsense-ros.git) into '~/ros2_ws/src/'
   ```bashrc
   git clone https://github.com/IntelRealSense/realsense-ros.git -b foxy
   cd ~/ros2_ws
   ```

  ### Step 4: Install dependencies:
   ```bash
  sudo apt-get install python3-rosdep -y
  sudo rosdep init
  rosdep update
  rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
  ```

  ### Step 5: Build:
  ```bash
  colcon build
  ```

  ### Step 6: Source (on each new terminal):
  ```bash
  . install/local_setup.bash
  ```


## Usage Instructions

### Start the camera node
To start the camera node in ROS:

```bash
  ros2 launch realsense2_camera rs_launch.py
```
or, with parameters specified in rs_launch.py, for example - pointcloud enabled:
```bash
ros2 launch realsense2_camera rs_launch.py enable_pointcloud:=true device_type:=d435
```
or, without using the supplement launch files:
```bash
ros2 run realsense2_camera realsense2_camera_node --ros-args -p filters:=colorizer
```


This will stream all camera sensors and publish on the appropriate ROS topics.

### Published Topics
The published topics differ according to the device and parameters.
After running the above command with D435i attached, the following list of topics will be available (This is a partial list. For full one type `ros2 topic list`):
- /camera/accel/imu_info
- /camera/color/camera_info
- /camera/color/image_raw
- /camera/depth/camera_info
- /camera/depth/color/points
- /camera/depth/image_rect_raw
- /camera/extrinsics/depth_to_color
- /camera/extrinsics/depth_to_infra1
- /camera/extrinsics/depth_to_infra2
- /camera/gyro/imu_info
- /camera/imu
- /camera/infra1/camera_info
- /camera/infra1/image_rect_raw
- /camera/infra2/camera_info
- /camera/infra2/image_rect_raw
- /camera/parameter_events
- /camera/rosout
- /parameter_events
- /rosout
- /tf_static

>Using an L515 device the list differs a little by adding a 4-bit confidence grade (published as a mono8 image):
>- /camera/confidence/camera_info
>- /camera/confidence/image_rect_raw
>
>It also replaces the 2 infrared topic sets with the single available one:
>- /camera/infra/camera_info
>- /camera/infra/image_raw

The "/camera" prefix is the namesapce specified in the given launch file.
When using D435 or D415, the gyro and accel topics wont be available. Likewise, other topics will be available when using T265 (see below).

### Available Parameters:
For the entire list of parameters type `ros2 param list`.

- **serial_no**: will attach to the device with the given serial number (*serial_no*) number. Default, attach to the first (in an inner list) RealSense device.
  - Note: serial number can also be defined with "_" prefix. For instance, serial number 831612073525 can be set in command line as `serial_no:=_831612073525`. That is a workaround until a better method will be found to ROS2's auto conversion of strings containing only digits into integers.
- **usb_port_id**: will attach to the device with the given USB port (*usb_port_id*). i.e 4-1, 4-2 etc. Default, ignore USB port when choosing a device.
- **device_type**: will attach to a device whose name includes the given *device_type* regular expression pattern. Default, ignore device type. For example, device_type:=d435 will match d435 and d435i. device_type=d435(?!i) will match d435 but not d435i.

- **rosbag_filename**: Will publish topics from rosbag file.
- **initial_reset**: On occasions the device was not closed properly and due to firmware issues needs to reset. If set to true, the device will reset prior to usage.
- **align_depth**: If set to true, will publish additional topics with the all the images aligned to the depth image.</br>
The topics are of the form: ```/camera/aligned_depth_to_color/image_raw``` etc.
- **filters**: any of the following options, separated by commas:</br>
 - ```colorizer```: will color the depth image. On the depth topic an RGB image will be published, instead of the 16bit depth values .
 - ```pointcloud```: will add a pointcloud topic `/camera/depth/color/points`.
    * The texture of the pointcloud can be modified in rqt_reconfigure (see below) or using the parameters: `pointcloud_texture_stream` and `pointcloud_texture_index`. Run rqt_reconfigure to see available values for these parameters.</br>
    * The depth FOV and the texture FOV are not similar. By default, pointcloud is limited to the section of depth containing the texture. You can have a full depth to pointcloud, coloring the regions beyond the texture with zeros, by setting `allow_no_texture_points` to true.
    * pointcloud is of an unordered format by default. This can be changed by setting `ordered_pc` to true.

 - The following filters have detailed descriptions in : https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md
   - ```disparity``` - convert depth to disparity before applying other filters and back.
   - ```spatial``` - filter the depth image spatially.
   - ```temporal``` - filter the depth image temporally.
   - ```hole_filling``` - apply hole-filling filter.
   - ```decimation``` - reduces depth scene complexity.
- **enable_sync**: gathers closest frames of different sensors, infra red, color and depth, to be sent with the same timetag. This happens automatically when such filters as pointcloud are enabled.
- ***<stream_type>*_width**, ***<stream_type>*_height**, ***<stream_type>*_fps**: <stream_type> can be any of *infra, color, fisheye, depth, gyro, accel, pose*. Sets the required format of the device. If the specified combination of parameters is not available by the device, the stream will be replaced with the default for that stream. Setting a value to 0, will choose the first format in the inner list. (i.e. consistent between runs but not defined).</br>*Note: for gyro accel and pose, only _fps option is meaningful.
- ***<stream_type>*_qos**: <stream_type> can be any of *infra, color, fisheye, depth, gyro, accel, pose*. Sets the QoS by which the topic is published. Available values are the following strings: SYSTEM_DEFAULT, PARAMETER_EVENTS, SERVICES_DEFAULT, PARAMETERS, DEFAULT, SENSOR_DATA.
- **enable_*<stream_name>***: Choose whether to enable a specified stream or not. Default is true for images and false for orientation streams. <stream_name> can be any of *infra1, infra2, color, depth, fisheye, fisheye1, fisheye2, gyro, accel, pose*.

- ***<stream_name>*_frame_id**, ***<stream_name>*_optical_frame_id**, **aligned_depth_to_*<stream_name>*_frame_id**: Specify the different frame_id for the different frames. Especially important when using multiple cameras.

- **base_frame_id**: defines the frame_id all static transformations refers to.
- **odom_frame_id**: defines the origin coordinate system in ROS convention (X-Forward, Y-Left, Z-Up). pose topic defines the pose relative to that system.

- **unite_imu_method**: The D435i and T265 cameras have built in IMU components which produce 2 unrelated streams: *gyro* - which shows angular velocity and *accel* which shows linear acceleration. Each with it's own frequency. By default, 2 corresponding topics are available, each with only the relevant fields of the message sensor_msgs::Imu are filled out.
Setting *unite_imu_method* creates a new topic, *imu*, that replaces the default *gyro* and *accel* topics. The *imu* topic is published at the rate of the gyro. All the fields of the Imu message under the *imu* topic are filled out.
   - **linear_interpolation**: Every gyro message is attached by the an accel message interpolated to the gyro's timestamp.
   - **copy**: Every gyro message is attached by the last accel message.
- **clip_distance**: remove from the depth image all values above a given value (meters). Disable by giving negative value (default)
- **linear_accel_cov**, **angular_velocity_cov**: sets the variance given to the Imu readings. For the T265, these values are being modified by the inner confidence value.
- **hold_back_imu_for_frames**: Images processing takes time. Therefor there is a time gap between the moment the image arrives at the wrapper and the moment the image is published to the ROS environment. During this time, Imu messages keep on arriving and a situation is created where an image with earlier timestamp is published after Imu message with later timestamp. If that is a problem, setting *hold_back_imu_for_frames* to *true* will hold the Imu messages back while processing the images and then publish them all in a burst, thus keeping the order of publication as the order of arrival. Note that in either case, the timestamp in each message's header reflects the time of it's origin.
- **topic_odom_in**: For T265, add wheel odometry information through this topic. The code refers only to the *twist.linear* field in the message.
- **calib_odom_file**: For the T265 to include odometry input, it must be given a [configuration file](https://github.com/IntelRealSense/librealsense/blob/master/unit-tests/resources/calibration_odometry.json). Explanations can be found [here](https://github.com/IntelRealSense/librealsense/pull/3462). The calibration is done in ROS coordinates system.
- **publish_tf**: boolean, publish or not TF at all. Defaults to True.
- **tf_publish_rate**: double, positive values mean dynamic transform publication with specified rate, all other values mean static transform publication. Defaults to 0 
- **publish_odom_tf**: If True (default) publish TF from odom_frame to pose_frame.
- **infra_rgb**: When set to True (default: False), it configures the infrared camera to stream in RGB (color) mode, thus enabling the use of a RGB image in the same frame as the depth image, potentially avoiding frame transformation related errors. When this feature is required, you are additionally required to also enable `enable_infra:=true` for the infrared stream to be enabled.
  - **NOTE** The configuration required for `enable_infra` is independent of `enable_depth`
  - **NOTE** To enable the Infrared stream, you should enable `enable_infra:=true` NOT `enable_infra1:=true` nor `enable_infra2:=true`
  - **NOTE** This feature is only supported by Realsense sensors with RGB streams available from the `infra` cameras, which can be checked by observing the output of `rs-enumerate-devices`

## Using T265 ##

### Start the camera node
To start the camera node in ROS:

```bash
ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_pose:=true -p device_type:=t265 -p fisheye_width:=848 -p fisheye_height:=800
```
or, if you also have a d4xx connected, you can try out the launch file:
```bash
ros2 launch realsense2_camera rs_d400_and_t265_launch.py enable_fisheye12:=true enable_fisheye22:=true
```
- note: the parameters are called `enable_fisheye12` and `enable_fisheye22`. The node knows them as `enable_fisheye1` and `enable_fisheye2` but launch file runs 2 nodes and these parameters refer to the second one.

## Still on the pipelie:
* Support ROS2 life cycle.
* Enable and disable sensors and filters.

### Unit tests:
Unit-tests are based on bag files saved on S3 server. These can be downloaded using the following commands:
```bash
cd ros2_ws
wget "http://realsense-hw-public.s3.amazonaws.com/rs-tests/TestData/outdoors.bag" -P "records/"
wget "http://realsense-hw-public.s3-eu-west-1.amazonaws.com/rs-tests/D435i_Depth_and_IMU_Stands_still.bag" -P "records/"
```
Then, unit-tests can be run using the following command (use either python or python3):
```bash
python3 src/realsense-ros/realsense2_camera/scripts/rs2_test.py --all
```

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
