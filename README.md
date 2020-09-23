# ROS2 Wrapper for Intel&reg; RealSense&trade; Devices
These are packages for using Intel RealSense cameras (D400 and L500 series, SR300 camera and T265 Tracking Module) with ROS2.

LibRealSense supported version: v2.38.1 (see [realsense2_camera release notes](https://github.com/IntelRealSense/realsense-ros/releases))

## Installation Instructions
This version supports ROS2 eloquent on Ubuntu 18.04.

   ### Step 1: Install the ROS2 distribution
   - #### Install [ROS2 Eloquent](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/), on Ubuntu 18.04.

   ```bash
  ROS_DISTRO=eloquent
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
   - #### Build from sources by downloading the latest [Intel&reg; RealSense&trade; SDK 2.0](https://github.com/IntelRealSense/librealsense/releases/tag/v2.38.1) and follow the instructions under [Linux Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)


   ### Step 3: Install Intel&reg; RealSense&trade; ROS2 wrapper from Sources
   - Create a ROS2 workspace
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src/
   ```
   - Clone the latest Eloquent Intel&reg; RealSense&trade;  wrapper from [here](https://github.com/IntelRealSense/realsense-ros.git) into '~/ros2_ws/src/'
   ```bashrc
   git clone https://github.com/IntelRealSense/realsense-ros.git -b eloquent
   cd ~/ros2_ws
   ```

  ### Step 4: Install dependencies:
   ```bash
  sudo apt-get install python-rosdep -y
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
  ros2 run realsense2_node realsense2_node 
```
or, with parameters, for example - pointcloud enabled:
```bash
ros2 run realsense2_node realsense2_node --ros-args -p filters:=pointcloud -p 
```

This will stream all camera sensors and publish on the appropriate ROS topics.

### Published Topics
The published topics differ according to the device and parameters.
After running the above command with D435i attached, the following list of topics will be available (This is a partial list. For full one type `ros2 topic list`):
- /camera1/accel/imu_info
- /camera1/color/camera_info
- /camera1/color/image_raw
- /camera1/depth/camera_info
- /camera1/depth/color/points
- /camera1/depth/image_rect_raw
- /camera1/extrinsics/depth_to_color
- /camera1/extrinsics/depth_to_infra1
- /camera1/extrinsics/depth_to_infra2
- /camera1/gyro/imu_info
- /camera1/imu
- /camera1/infra1/camera_info
- /camera1/infra1/image_rect_raw
- /camera1/infra2/camera_info
- /camera1/infra2/image_rect_raw
- /camera1/parameter_events
- /camera1/rosout
- /parameter_events
- /rosout
- /tf_static

The "/camera1" prefix is the namesapce specified in the given launch file.
When using D435 or D415, the gyro and accel topics wont be available. Likewise, other topics will be available when using T265 (see below).

### Available Parameters:
For the entire list of parameters type `ros2 param list`.

- **serial_no**: will attach to the device with the given serial number (*serial_no*) number. Default, attach to available RealSense device in random.
- **usb_port_id**: will attach to the device with the given USB port (*usb_port_id*). i.e 4-1, 4-2 etc. Default, ignore USB port when choosing a device.
- **device_type**: will attach to a device whose name includes the given *device_type* regular expression pattern. Default, ignore device type. For example, device_type:=d435 will match d435 and d435i. device_type=d435(?!i) will match d435 but not d435i.

- **rosbag_filename**: Will publish topics from rosbag file.
- **initial_reset**: On occasions the device was not closed properly and due to firmware issues needs to reset. If set to true, the device will reset prior to usage.
- **align_depth**: If set to true, will publish additional topics with the all the images aligned to the depth image.</br>
The topics are of the form: ```/camera/aligned_depth_to_color/image_raw``` etc.
- **filters**: any of the following options, separated by commas:</br>
 - ```colorizer```: will color the depth image. On the depth topic an RGB image will be published, instead of the 16bit depth values .
 - ```pointcloud```: will add a pointcloud topic `/camera/depth/color/points`. The texture of the pointcloud can be modified in rqt_reconfigure (see below) or using the parameters: `pointcloud_texture_stream` and `pointcloud_texture_index`. Run rqt_reconfigure to see available values for these parameters.</br>
 The depth FOV and the texture FOV are not similar. By default, pointcloud is limited to the section of depth containing the texture. You can have a full depth to pointcloud, coloring the regions beyond the texture with zeros, by setting `allow_no_texture_points` to true.

 - The following filters have detailed descriptions in : https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md
   - ```disparity``` - convert depth to disparity before applying other filters and back.
   - ```spatial``` - filter the depth image spatially.
   - ```temporal``` - filter the depth image temporally.
   - ```hole_filling``` - apply hole-filling filter.
   - ```decimation``` - reduces depth scene complexity.
- **enable_sync**: gathers closest frames of different sensors, infra red, color and depth, to be sent with the same timetag. This happens automatically when such filters as pointcloud are enabled.
- ***<stream_type>*_width**, ***<stream_type>*_height**, ***<stream_type>*_fps**: <stream_type> can be any of *infra, color, fisheye, depth, gyro, accel, pose*. Sets the required format of the device. If the specified combination of parameters is not available by the device, the stream will not be published. Setting a value to 0, will choose the first format in the inner list. (i.e. consistent between runs but not defined). Note: for gyro accel and pose, only _fps option is meaningful.
- **enable_*<stream_name>***: Choose whether to enable a specified stream or not. Default is true. <stream_name> can be any of *infra1, infra2, color, depth, fisheye, fisheye1, fisheye2, gyro, accel, pose*.

- ***<stream_name>*_frame_id**: Choose whether to enable a specified stream or not. Default is true. <stream_name> can be any of *infra1, infra2, color, depth, fisheye, fisheye1, fisheye2, gyro, accel, pose*.


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

## Using T265 ##
**Important Notice:** For wheeled robots, odometer input is a requirement for robust and accurate tracking. The relevant APIs will be added to librealsense and ROS/realsense in upcoming releases. Currently, the API is available in the [underlying device driver](https://github.com/IntelRealSense/librealsense/blob/master/third-party/libtm/libtm/include/TrackingDevice.h#L508-L515).

### Start the camera node
To start the camera node in ROS:

```bash
ros2 run realsense2_node realsense2_node --ros-args -p enable_pose:=true -p device_type:=t265 -p fisheye_width:=848 -p fisheye_height:=800
```

## Still on the pipelie:
* Launch files for running multiple cameras.
* Import descriptive files (realsense2_description package).
* Support ROS2 life cycle.
* Enable and disable sensors and filters.


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
