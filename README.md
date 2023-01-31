# ROS2 Wrapper for Intel&reg; RealSense&trade; Devices
These are packages for using Intel RealSense cameras (D400, L500 and SR300 cameras) with ROS2.

This version supports ROS2 Dashing, Eloquent, Foxy, Galactic and Rolling.

LibRealSense supported version: v2.51.1 (see [realsense2_camera release notes](https://github.com/IntelRealSense/realsense-ros/releases))


## For LibRS ROS1 Wrapper please refer to [ROS1-legacy branch](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy)

## Please notice: if you are moving from RealSense [ROS2-legacy branch](https://github.com/IntelRealSense/realsense-ros/tree/ros2-legacy) to ROS2-development:
- **Changed Parameters**:
    - **"stereo_module"**, **"l500_depth_sensor"** are replaced by **"depth_module"**
    - For video streams: **\<module>.profile** replaces **\<stream>_width**, **\<stream>_height**, **\<stream>_fps**
        - **ROS2-legacy (Old)**:
          - ros2 launch realsense2_camera rs_launch.py depth_width:=640 depth_height:=480 depth_fps:=30.0 infra1_width:=640 infra1_height:=480 infra1_fps:=30.0
        - **ROS2-development (New)**:
          - ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30
    - Removed paramets **\<stream>_frame_id**, **\<stream>_optical_frame_id**. frame_ids are now defined by camera_name
    - **"filters"** is removed. All filters (or post-processing blocks) are enabled/disabled using **"\<filter>.enable"**
    - **"align_depth"** is now a regular processing block and as such the parameter for enabling it is replaced with **"align_depth.enable"**
    - **"allow_no_texture_points"**, **"ordered_pc"** are now belong to the pointcloud filter and as such are replaced by **"pointcloud.allow_no_texture_points"**, **"pointcloud.ordered_pc"**
    - **"pointcloud_texture_stream"**, **"pointcloud_texture_index"** belong now to the pointcloud filter and were renamed to match their librealsense' names: **"pointcloud.stream_filter"**, **"pointcloud.stream_index_filter"**
- Allow enable/disable of sensors in runtime (parameters **\<stream>.enable**)
- Allow enable/disable of filters in runtime (parameters **\<filter_name>.enable**)
- **unite_imu_method** parameter is now changeable in runtime.
- **enable_sync** parameter is now changeable in runtime.



## Installation Instructions

   ### Step 1: Install the ROS2 distribution
 - #### Ubuntu 22.04:
   - [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
 - #### Ubuntu 20.04: 
   - [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
   - [ROS2 Galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
 - #### Ubuntu 18.04 : 
   - [ROS2 Dashing](https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Debians.html)
   - [ROS2 Eloquent](https://docs.ros.org/en/eloquent/Installation/Linux-Install-Debians.html)


### Step 2: Install the latest Intel&reg; RealSense&trade; SDK 2.0

- #### Option 1: Install librealsense2 debian package (Not supported in Ubuntu 22.04 yet)
   - Jetson users - use the [Jetson Installation Guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md)
   - Otherwise, install from [Linux Debian Installation Guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)
      - In this case treat yourself as a developer: make sure to follow the instructions to also install librealsense2-dev and librealsense2-dkms packages

- #### Option 2: Build from source
  - Download the latest [Intel&reg; RealSense&trade; SDK 2.0](https://github.com/IntelRealSense/librealsense/releases/tag/v2.51.1)
  - Follow the instructions under [Linux Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)


### Step 3: Install Intel&reg; RealSense&trade; ROS2 wrapper from sources
   - Create a ROS2 workspace
      ```bash
      mkdir -p ~/ros2_ws/src
      cd ~/ros2_ws/src/
      ```
   - Clone the latest ROS2 Intel&reg; RealSense&trade;  wrapper from [here](https://github.com/IntelRealSense/realsense-ros.git) into '~/ros2_ws/src/'
      ```bashrc
      git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
      cd ~/ros2_ws
      ```
### Step 4: Install dependencies
   ```bash
   sudo apt-get install python3-rosdep -y
   sudo rosdep init # "sudo rosdep init --include-eol-distros" for Eloquent and earlier
   rosdep update # "sudo rosdep update --include-eol-distros" for Eloquent and earlier
   rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
   ```

### Step 5: Build
   ```bash
   colcon build
   ```

### Step 6: Terminal environment
   ```bash
   ROS_DISTRO=<YOUR_SYSTEM_ROS_DISTRO>  # set your ROS_DISTRO: humble, galactic, foxy, eloquent, dashing
   source /opt/ros/$ROS_DISTRO/setup.bash
   cd ~/ros2_ws
   . install/local_setup.bash
   ```

&nbsp;

## Usage Instructions

### Start the camera node
To start the camera node in ROS:

```bash
ros2 launch realsense2_camera rs_launch.py
```
or, with parameters, for example - temporal and spatial filters are enabled:
```bash
ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_color:=false -p spatial_filter.enable:=true -p temporal_filter.enable:=true
```

or, with a launch file:
```bash
ros2 launch realsense2_camera rs_launch.py
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=true
```

This will stream all camera sensors and publish on the appropriate ROS topics.

### Published Topics
The published topics differ according to the device and parameters.
After running the above command with D435i attached, the following list of topics will be available (This is a partial list. For full one type `ros2 topic list`):
- /camera/aligned_depth_to_color/camera_info
- /camera/aligned_depth_to_color/image_raw
- /camera/color/camera_info
- /camera/color/image_raw
- /camera/color/metadata
- /camera/depth/camera_info
- /camera/depth/color/points
- /camera/depth/image_rect_raw
- /camera/depth/metadata
- /camera/extrinsics/depth_to_color
- /camera/imu
- /diagnostics
- /parameter_events
- /rosout
- /tf_static


Enabling accel and gyro is achieved either by adding the following parameters to the command line:</br>
`ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true enable_gyro:=true enable_accel:=true` </br>
or in runtime using the following commands:
```
ros2 param set /camera/camera enable_accel true
ros2 param set /camera/camera enable_gyro true
```

Enabling stream adds matching topics. For instance, enabling the gyro and accel streams adds the following topics:
- /camera/accel/imu_info
- /camera/accel/metadata
- /camera/accel/sample
- /camera/extrinsics/depth_to_accel
- /camera/extrinsics/depth_to_gyro
- /camera/gyro/imu_info
- /camera/gyro/metadata
- /camera/gyro/sample


>Using an L515 device the list differs a little by adding a 4-bit confidence grade (published as a mono8 image):
>- /camera/confidence/camera_info
>- /camera/confidence/image_rect_raw
>
>It also replaces the 2 infrared topic sets with the single available one:
>- /camera/infra/camera_info
>- /camera/infra/image_raw

To turn them off: `ros2 param set /camera/camera enable_infra false`
The "/camera" prefix is the namesapce specified in the given launch file.
When using D435 or D415, the gyro and accel topics wont be available.

### The metadata topic:
The metadata messages store the camera's available metadata in a *json* format. To learn more, a dedicated script for echoing a metadata topic in runtime is attached. For instance, use the following command to echo the camera/depth/metadata topic:
```
python3 src/realsense-ros/realsense2_camera/scripts/echo_metadada.py /camera/depth/metadata
```

### Post processing blocks - i.e. filters:
The following processing blocks are available:
 - ```align_depth```: If enabled, will publish the depth image aligned to the color image on the topic `/camera/aligned_depth_to_color/image_raw`.</br> The pointcloud, if created, will be based on the aligned depth image.

 - ```colorizer```: will color the depth image. On the depth topic an RGB image will be published, instead of the 16bit depth values .
 - ```pointcloud```: will add a pointcloud topic `/camera/depth/color/points`.
    * The texture of the pointcloud can be modified using the `pointcloud.stream_filter` parameter.</br>
    * The depth FOV and the texture FOV are not similar. By default, pointcloud is limited to the section of depth containing the texture. You can have a full depth to pointcloud, coloring the regions beyond the texture with zeros, by setting `pointcloud.allow_no_texture_points` to true.
    * pointcloud is of an unordered format by default. This can be changed by setting `pointcloud.ordered_pc` to true.

 - ```hdr_merge```: Allows depth image to be created by merging the information from 2 consecutive frames, taken with different exposure and gain values. The way to set exposure and gain values for each sequence in runtime is by first selecting the sequence id, using the `depth_module.sequence_id` parameter and then modifying the `depth_module.gain`, and `depth_module.exposure`.</br> To view the effect on the infrared image for each sequence id use the `sequence_id_filter.sequence_id` parameter.</br> To initialize these parameters in start time use the following parameters:</br>
  `depth_module.exposure.1`, `depth_module.gain.1`, `depth_module.exposure.2`, `depth_module.gain.2`</br>
  \* For in-depth review of the subject please read the accompanying [white paper](https://dev.intelrealsense.com/docs/high-dynamic-range-with-stereoscopic-depth-cameras).

 - The following filters have detailed descriptions in : https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md
   - ```disparity_filter``` - convert depth to disparity before applying other filters and back.
   - ```spatial_filter``` - filter the depth image spatially.
   - ```temporal_filter``` - filter the depth image temporally.
   - ```hole_filling_filter``` - apply hole-filling filter.
   - ```decimation_filter``` - reduces depth scene complexity.

Each of the above filters have it's own parameters, following the naming convention of `<filter_name>.<parameter_name>` including a `<filter_name>.enable` parameter to enable/disable it. 

### Sensor Parameters:
Each sensor has a unique set of parameters.
Video sensors, such as depth_module or rgb_camera have, at least, the 'profile' parameter.</br>
It is a string of the following format: \<width>X\<height>X\<fps> (The deviding character can be X, x or ",". Spaces are ignored.)

Since infra1, infra2 and depth are all streams of the depth_module, their width, height and fps are defined by their common sensor.
The same rule applies in L515 for the depth, infra and confidence streams which all share the parameters of their common depth_module.
If the specified combination of parameters is not available by the device, the default configuration will be used.</br>

### Available Parameters:
For the entire list of parameters type `ros2 param list`.
For reading a parameter value use `ros2 param get <node> <parameter_name>` for instance: `ros2 param get /camera/camera depth_module.emitter_on_off`
For setting a new value for a parameter use `ros2 param set <node> <parameter_name> <value>` i.e. `ros2 param set /camera/camera depth_module.emitter_on_off true`

#### Parameters that can be modified during runtime:
- All of the filters and sensors inner parameters.
- **enable_*<stream_name>***: Choose whether to enable a specified stream or not. Default is true for images and false for orientation streams. <stream_name> can be any of *infra1, infra2, color, depth, fisheye, fisheye1, fisheye2, gyro, accel, pose*.
- **enable_sync**: gathers closest frames of different sensors, infra red, color and depth, to be sent with the same timetag. This happens automatically when such filters as pointcloud are enabled.
- ***<stream_type>*_qos**: <stream_type> can be any of *infra, color, fisheye, depth, gyro, accel, pose*. Sets the QoS by which the topic is published. Available values are the following strings: SYSTEM_DEFAULT, DEFAULT, PARAMETER_EVENTS, SERVICES_DEFAULT, PARAMETERS, SENSOR_DATA.
- **Notice:** ***<stream_type>*_info_qos** refers to both camera_info topics and metadata topics.
- **tf_publish_rate**: double, positive values mean dynamic transform publication with specified rate, all other values mean static transform publication. Defaults to 0 



#### Parameters that cannot be changed in runtime:
- **serial_no**: will attach to the device with the given serial number (*serial_no*) number. Default, attach to the first (in an inner list) RealSense device.
  - Note: serial number can also be defined with "_" prefix. For instance, serial number 831612073525 can be set in command line as `serial_no:=_831612073525`. That is a workaround until a better method will be found to ROS2's auto conversion of strings containing only digits into integers.
- **usb_port_id**: will attach to the device with the given USB port (*usb_port_id*). i.e 4-1, 4-2 etc. Default, ignore USB port when choosing a device.
- **device_type**: will attach to a device whose name includes the given *device_type* regular expression pattern. Default, ignore device type. For example, device_type:=d435 will match d435 and d435i. device_type=d435(?!i) will match d435 but not d435i.
- **reconnect_timeout**: When the driver cannot connect to the device try to reconnect after this timeout (in seconds).
- **wait_for_device_timeout**: If the specified device is not found, will wait *wait_for_device_timeout* seconds before exits. Defualt, *wait_for_device_timeout < 0*, will wait indefinitely.
- **rosbag_filename**: Publish topics from rosbag file. There are two ways for loading rosbag file:
   * Command line - ```ros2 run realsense2_camera realsense2_camera_node -p rosbag_filename:="/full/path/to/rosbag.bag"```
   * Launch file - set ```rosbag_filename``` parameter with rosbag full path (see ```realsense2_camera/launch/rs_launch.py``` as reference) 

- **initial_reset**: On occasions the device was not closed properly and due to firmware issues needs to reset. If set to true, the device will reset prior to usage.

- ***<stream_name>*_frame_id**, ***<stream_name>*_optical_frame_id**, **aligned_depth_to_*<stream_name>*_frame_id**: Specify the different frame_id for the different frames. Especially important when using multiple cameras.

- **base_frame_id**: defines the frame_id all static transformations refers to.
- **odom_frame_id**: defines the origin coordinate system in ROS convention (X-Forward, Y-Left, Z-Up). pose topic defines the pose relative to that system.

- **unite_imu_method**: The D435i camera has built in IMU components which produce 2 unrelated streams: *gyro* - which shows angular velocity and *accel* which shows linear acceleration. Each with it's own frequency. By default, 2 corresponding topics are available, each with only the relevant fields of the message sensor_msgs::Imu are filled out.
Setting *unite_imu_method* creates a new topic, *imu*, that replaces the default *gyro* and *accel* topics. The *imu* topic is published at the rate of the gyro. All the fields of the Imu message under the *imu* topic are filled out. `unite_imu_method` parameter supported values are [0-2] meaning:  [0 -> None, 1 -> Copy, 2 -> Linear_ interpolation] when:
   - **linear_interpolation**: Every gyro message is attached by the an accel message interpolated to the gyro's timestamp.
   - **copy**: Every gyro message is attached by the last accel message.
- **clip_distance**: remove from the depth image all values above a given value (meters). Disable by giving negative value (default)
- **linear_accel_cov**, **angular_velocity_cov**: sets the variance given to the Imu readings.
- **hold_back_imu_for_frames**: Images processing takes time. Therefor there is a time gap between the moment the image arrives at the wrapper and the moment the image is published to the ROS environment. During this time, Imu messages keep on arriving and a situation is created where an image with earlier timestamp is published after Imu message with later timestamp. If that is a problem, setting *hold_back_imu_for_frames* to *true* will hold the Imu messages back while processing the images and then publish them all in a burst, thus keeping the order of publication as the order of arrival. Note that in either case, the timestamp in each message's header reflects the time of it's origin.
- **publish_tf**: boolean, publish or not TF at all. Defaults to True.
- **diagnostics_period**: double, positive values set the period between diagnostics updates on the `/diagnostics` topic. 0 or negative values mean no diagnostics topic is published. Defaults to 0.</br>
The `/diagnostics` topic includes information regarding the device temperatures and actual frequency of the enabled streams.

- **publish_odom_tf**: If True (default) publish TF from odom_frame to pose_frame.

### Available services:
- device_info : retrieve information about the device - serial_number, firmware_version etc. Type `ros2 interface show realsense2_camera_msgs/srv/DeviceInfo` for the full list. Call example: `ros2 service call /camera/device_info realsense2_camera_msgs/srv/DeviceInfo`
  - Note that for **ROS2 Dashing** the command is `ros2 srv show realsense2_camera_msgs/srv/DeviceInfo`


## Efficient intra-process communication:

Our ROS2 Wrapper node supports zero-copy communications if loaded in the same process as a subscriber node. This can reduce copy times on image topics (not point-cloud or others), especially with big frame resolutions and high FPS.

You will need to launch a component container and launch our node as a component together with other component nodes. Further details on "Composing multiple nodes in a single process" can be found [here](https://docs.ros.org/en/rolling/Tutorials/Composition.html).

Further details on efficient intra-process communication can be found [here](https://docs.ros.org/en/foxy/Tutorials/Intra-Process-Communication.html#efficient-intra-process-communication).

### Example
#### Manually loading multiple components into the same process
* Start the component:
  ```bash
  ros2 run rclcpp_components component_container
  ```

* Add the wrapper:
  ```bash
  ros2 component load /ComponentManager realsense2_camera realsense2_camera::RealSenseNodeFactory -e use_intra_process_comms:=true
  ```
  Load other component nodes (consumers of the wrapper topics) in the same way.

### Limitations

* Node components are currently not supported on RCLPY
* Compressed images using `image_transport` will be disabled as this isn't supported with intra-process communication

### Latency test tool and launch file

For getting a sense of the latency reduction, a frame latency reporter tool is available via a launch file.
The launch file loads the wrapper and a frame latency reporter tool component into a single container (so the same process).
The tool prints out the frame latency (`now - frame.timestamp`) per frame.

The tool is not built unless asked for. Turn on `BUILD_TOOLS` during build to have it available:
```bash
colcon build --cmake-args '-DBUILD_TOOLS=ON'
```

The launch file accepts a parameter, `intra_process_comms`, controlling whether zero-copy is turned on or not. Default is on:
```bash
ros2 launch realsense2_camera rs_intra_process_demo_launch.py intra_process_comms:=true
```


## Still in the pipeline:


* Migrate infra_rgb option.

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
Copyright 2022 Intel Corporation

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
