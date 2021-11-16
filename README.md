# ROS2 Wrapper for Intel&reg; RealSense&trade; Devices
These are packages for using Intel RealSense cameras (D400 and L500 series, SR300 camera and T265 Tracking Module) with ROS2.

LibRealSense supported version: v2.50.0 (see [realsense2_camera release notes](https://github.com/IntelRealSense/realsense-ros/releases))

## Installation Instructions
This version supports ROS2 Dashing, Eloquent and Foxy.

   ### Step 1: Install the ROS2 distribution
   - #### Install [ROS2 Dashing](https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Debians.html), on Ubuntu 18.04 or [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) or [ROS2 Galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html), on Ubuntu 20.04

### There are 2 sources to install realsense2_camera from:

* ### Method 1: The ROS distribution:

  *Ubuntu*

    realsense2_camera is available as a debian package of ROS distribution. It can be installed by typing:
    
    ```sudo apt-get install ros-$ROS_DISTRO-realsense2-camera```

    This will install both realsense2_camera and its dependents, including librealsense2 library and matching udev-rules.

    Notice:
    * The version of librealsense2 is almost always behind the one availeable in RealSense&trade; official repository.
    * librealsense2 is not built to use native v4l2 driver but the less stable RS-USB protocol. That is because the RS-USB protocol is more general and operational on a larger variety of platforms. This have limitations when running multiple cameras and when using T265.
    * realsense2_description is available as a separate debian package of ROS distribution. It includes the 3D-models of the devices and is necessary for running launch files that include these models (i.e. view_model.launch.py). It can be installed by typing:
    `sudo apt-get install ros-$ROS_DISTRO-realsense2-description`

* ### Method 2: The RealSense&trade; distribution:
     > This option is demonstrated in the [.travis.yml](https://github.com/IntelRealSense/realsense-ros/blob/ros2/.travis.yml) file. It basically summerize the elaborate instructions in the following 2 steps:


   ### Step 1: Install the latest Intel&reg; RealSense&trade; SDK 2.0

    Install librealsense2 debian package:
    * Jetson users - use the [Jetson Installation Guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md)
    * Otherwise, install from [Linux Debian Installation Guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)
      - In that case treat yourself as a developer. Make sure you follow the instructions to also install librealsense2-dev and librealsense2-dkms packages.


   #### OR
   - #### Build from sources by downloading the latest [Intel&reg; RealSense&trade; SDK 2.0](https://github.com/IntelRealSense/librealsense/releases/tag/v2.50.0) and follow the instructions under [Linux Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)


   ### Step 3: Install Intel&reg; RealSense&trade; ROS2 wrapper from Sources
   - Create a ROS2 workspace
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src/
   ```
   - Clone the latest ROS2 Intel&reg; RealSense&trade;  wrapper from [here](https://github.com/IntelRealSense/realsense-ros.git) into '~/ros2_ws/src/'
   ```bashrc
   git clone --depth 1 --branch `git ls-remote --tags https://github.com/IntelRealSense/realsense-ros.git | grep -Po "(?<=tags/)3.\d+\.\d+" | sort -V | tail -1` https://github.com/IntelRealSense/realsense-ros.git
   cd ~/ros2_ws
   ```

  ### Step 4: Install dependencies:
   ```bash
  sudo apt-get install python3-rosdep -y
  sudo rosdep init # "sudo rosdep init --include-eol-distros" for Dashing
  rosdep update
  rosdep install -i --from-path src --rosdistro $_ros_dist --skip-keys=librealsense2 -y
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
or, with a demo config file:
```bash
ros2 launch realsense2_camera rs_launch.py config_file:="'$COLCON_PREFIX_PATH/realsense2_camera/share/realsense2_camera/config/d435i.yaml'"
```


This will stream all camera sensors and publish on the appropriate ROS topics.

### Published Topics
The published topics differ according to the device and parameters.
After running the above command with D435i attached, the following list of topics will be available (This is a partial list. For full one type `ros2 topic list`):
- /camera/accel/imu_info
- /camera/color/camera_info
- /camera/color/image_raw
- /camera/color/metadata
- /camera/depth/camera_info
- /camera/depth/color/points
- /camera/depth/image_rect_raw
- /camera/depth/metadata
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
- /diagnostics
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
- **reconnect_timeout**: When the driver cannot connect to the device try to reconnect after this timeout (in seconds).
- **wait_for_device_timeout**: If the specified device is not found, will wait *wait_for_device_timeout* seconds before exits. Defualt, *wait_for_device_timeout < 0*, will wait indefinitely.
- **rosbag_filename**: Will publish topics from rosbag file.
- **initial_reset**: On occasions the device was not closed properly and due to firmware issues needs to reset. If set to true, the device will reset prior to usage.
- **align_depth**: If set to true, will publish additional topics for the "aligned depth to color" image.: ```/camera/aligned_depth_to_color/image_raw```, ```/camera/aligned_depth_to_color/camera_info```.</br>
The pointcloud, if enabled, will be built based on the aligned_depth_to_color image.</br>
- **filters**: any of the following options, separated by commas:</br>
 - ```colorizer```: will color the depth image. On the depth topic an RGB image will be published, instead of the 16bit depth values .
 - ```pointcloud```: will add a pointcloud topic `/camera/depth/color/points`.
    * The texture of the pointcloud can be modified in rqt_reconfigure (see below) or using the parameters: `pointcloud_texture_stream` and `pointcloud_texture_index`. Run rqt_reconfigure to see available values for these parameters.</br>
    * The depth FOV and the texture FOV are not similar. By default, pointcloud is limited to the section of depth containing the texture. You can have a full depth to pointcloud, coloring the regions beyond the texture with zeros, by setting `allow_no_texture_points` to true.
    * pointcloud is of an unordered format by default. This can be changed by setting `ordered_pc` to true.
 - ```hdr_merge```: Allows depth image to be created by merging the information from 2 consecutive frames, taken with different exposure and gain values. The way to set exposure and gain values for each sequence in runtime is by first selecting the sequence id, using the `stereo_module.sequence_id` parameter and then modifying the `stereo_module.gain`, and `stereo_module.exposure`.</br> To view the effect on the infrared image for each sequence id use the `sequence_id_filter.sequence_id` parameter.</br> To initialize these parameters in start time use the following parameters:</br>
  `stereo_module.exposure.1`, `stereo_module.gain.1`, `stereo_module.exposure.2`, `stereo_module.gain.2`</br>
  \* For in-depth review of the subject please read the accompanying [white paper](https://dev.intelrealsense.com/docs/high-dynamic-range-with-stereoscopic-depth-cameras).

  
 - The following filters have detailed descriptions in : https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md
   - ```disparity``` - convert depth to disparity before applying other filters and back.
   - ```spatial``` - filter the depth image spatially.
   - ```temporal``` - filter the depth image temporally.
   - ```hole_filling``` - apply hole-filling filter.
   - ```decimation``` - reduces depth scene complexity.
- **enable_sync**: gathers closest frames of different sensors, infra red, color and depth, to be sent with the same timetag. This happens automatically when such filters as pointcloud are enabled.
- ***<stream_type>*_width**, ***<stream_type>*_height**, ***<stream_type>*_fps**: <stream_type> can be any of *infra, color, fisheye, depth, gyro, accel, pose, confidence*. Sets the required format of the device. If the specified combination of parameters is not available by the device, the stream will be replaced with the default for that stream. Setting a value to 0, will choose the first format in the inner list. (i.e. consistent between runs but not defined).</br>*Note: for gyro accel and pose, only _fps option is meaningful.
- ***<stream_type>*_qos**, ***<stream_type>*_info_qos**, and ***<stream_type>*_extrinsics_qos**: <stream_type> can be any of *infra, color, fisheye, depth, gyro, accel, pose, confidence, pointcloud, imu*. Sets the QoS by which the topic is published. Available values are the following strings: SYSTEM_DEFAULT, PARAMETER_EVENTS, SERVICES_DEFAULT, PARAMETERS, DEFAULT, SENSOR_DATA, HID_DEFAULT (= DEFAULT with depth of 100), EXTRINSICS_DEFAULT (= DEFAULT with depth of 1 and transient local durabilty).</br>
**Notice:** ***<stream_type>*_info_qos** refers to both camera_info topics and metadata topics.
- **enable_*<stream_name>***: Choose whether to enable a specified stream or not. Default is true for images and false for orientation streams. <stream_name> can be any of *infra1, infra2, color, depth, fisheye, fisheye1, fisheye2, gyro, accel, pose, confidence*.

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
- **diagnostics_period**: double, positive values set the period between diagnostics updates on the `/diagnostics` topic. 0 or negative values mean no diagnostics topic is published. Defaults to 0.
- **publish_odom_tf**: If True (default) publish TF from odom_frame to pose_frame.
- **infra_rgb**: When set to True (default: False), it configures the infrared camera to stream in RGB (color) mode, thus enabling the use of a RGB image in the same frame as the depth image, potentially avoiding frame transformation related errors. When this feature is required, you are additionally required to also enable `enable_infra:=true` for the infrared stream to be enabled.
  - **NOTE** The configuration required for `enable_infra` is independent of `enable_depth`
  - **NOTE** To enable the Infrared stream, you should enable `enable_infra:=true` NOT `enable_infra1:=true` nor `enable_infra2:=true`
  - **NOTE** This feature is only supported by Realsense sensors with RGB streams available from the `infra` cameras, which can be checked by observing the output of `rs-enumerate-devices`

### Available services:
- enable : Start/Stop all streaming sensors. Usage example: `ros2 service call /camera/enable std_srvs/srv/SetBool "{data: False}"`
- device_info : retrieve information about the device - serial_number, firmware_version etc. Type `ros2 interface show realsense2_camera_msgs/srv/DeviceInfo` for the full list. Call example: `ros2 service call /camera/device_info realsense2_camera_msgs/srv/DeviceInfo`

### Point Cloud
Here is an example of how to start the camera node and make it publish the point cloud using the pointcloud option.
```bash
ros2 launch realsense2_camera demo_pointcloud_launch.py
```
An rviz2 node opens to watch the pointcloud:
<p align="center"><img src="https://user-images.githubusercontent.com/40540281/122672460-42dd3f80-d1d4-11eb-8767-4e61a64ced5b.gif" /></p>


### Aligned Depth Frames
Here is an example of how to start the camera node and make it publish the aligned depth stream to color stream.
```bash
ros2 launch realsense2_camera rs_launch.py align_depth:=true
```
Looking at the published topics:

```bash
ros2 topic list
```

#### **/camera/aligned_depth_to_color/camera_info**
#### **/camera/aligned_depth_to_color/image_raw**
#### /camera/color/camera_info
#### /camera/color/image_raw
#### /camera/depth/camera_info
#### /camera/depth/image_rect_raw
#### /camera/extrinsics/depth_to_color
#### /parameter_events
#### /rosout
#### /tf
#### /tf_static


### View and Modify Camera Controls Params in runtime:
The following command allow to change camera control values.
```bash
ros2 run rqt_reconfigure rqt_reconfigure
```
<p align="center"><img src="https://user-images.githubusercontent.com/40540281/122672659-6f458b80-d1d5-11eb-9262-545d2073e1da.png" /></p>

### Work with multiple cameras
Every realsense2_camera node is an independent process. One can 2 nodes using a dedicated launch file:
```bash
ros2 launch realsense2_camera rs_multi_camera_launch.py camera_name1:=my_D435 device_type1:=d435 camera_name2:=my_d415 device_type2:=d415
```
or by specifying serial numbers and using default *camera1* and *camera2* node names:
```
ros2 launch realsense2_camera rs_multi_camera_launch.py serial_no1:=_036522070660 serial_no2:=_725112060349
```
or launch each from a separate terminal:
```bash
ros2 launch realsense2_camera rs_launch.py camera_name:=my_d415 serial_no:=_036522070660
```
and 
```bash
ros2 launch realsense2_camera rs_launch.py camera_name:=my_d435 serial_no:=_725112060349
```
Notice the importance of defining a different camera_name for each node as this is the header of the topics.


## Using T265 ##

### Start the camera node
To start the camera node in ROS:

```bash
ros2 launch realsense2_camera rs_t265_launch.py
```
or, if you also have a d4xx connected, you can try out the launch file:
```bash
ros2 launch realsense2_camera rs_d400_and_t265_launch.py enable_fisheye12:=true enable_fisheye22:=true
```
- note: the parameters are called `enable_fisheye12` and `enable_fisheye22`. The node knows them as `enable_fisheye1` and `enable_fisheye2` but launch file runs 2 nodes and these parameters refer to the second one.

To visualize the pose output and frames in RViz, start:
```bash
ros2 launch realsense2_camera demo_t265_launch.py
```

### realsense2_description package:
For viewing included models, a separate package, realsense2_description, is included. For example:
```bash
ros2 launch realsense2_description view_model.launch.py model:=test_d415_camera.urdf.xacro
```
or, to list available models:
```
ros2 launch realsense2_description view_model.launch.py
```

## Still on the pipeline:
* Support ROS2 life cycle.
* Enable and disable sensors and filters.

### Unit tests:
Unit-tests are based on bag files saved on S3 server. These can be downloaded using the following commands:
```bash
cd ros2_ws
wget "https://librealsense.intel.com/rs-tests/TestData/outdoors_1color.bag" -P "records/"
wget "https://librealsense.intel.com/rs-tests/D435i_Depth_and_IMU_Stands_still.bag" -P "records/"
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
