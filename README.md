<h1 align="center">
   <img src="https://www.intelrealsense.com/wp-content/uploads/2020/09/intel-realsense-logo-360px.png" alt="Intel® RealSense™" title="Intel® RealSense™" />
</h1>

<p align="center">
  ROS2 packages for using Intel RealSense D400 cameras.<br>
  <a href="https://github.com/IntelRealSense/realsense-ros/releases">Latest release notes</a>
</p>

<hr>


[![rolling][rolling-badge]][rolling]
[![iron][iron-badge]][iron]
[![humble][humble-badge]][humble]
[![ubuntu22][ubuntu22-badge]][ubuntu22]
[![ubuntu20][ubuntu20-badge]][ubuntu20]

![GitHubWorkflowStatus](https://img.shields.io/github/actions/workflow/status/IntelRealSense/realsense-ros/main.yml?logo=github&style=flat-square)
[![GitHubcontributors](https://img.shields.io/github/contributors/IntelRealSense/realsense-ros?style=flat-square)](CONTRIBUTING.md)
[![License](https://img.shields.io/github/license/IntelRealSense/realsense-ros?style=flat-square)](LICENSE)

<hr>

## Table of contents
  * [ROS1 and ROS2 legacy](#ros1-and-ros2-legacy)
  * [Installation](#installation)
  * [Usage](#usage)
     * [Starting the camera node](#start-the-camera-node)
     * [Camera name and namespace](#camera-name-and-camera-namespace)
     * [Parameters](#parameters)
     * [ROS2-vs-Optical Coordination Systems](#ros2robot-vs-opticalcamera-coordination-systems)
     * [TF from coordinate A to coordinate B](#tf-from-coordinate-a-to-coordinate-b)
     * [Extrinsics from sensor A to sensor B](#extrinsics-from-sensor-a-to-sensor-b)
     * [Topics](#published-topics)
     * [RGBD Topic](#rgbd-topic)
     * [Metadata Topic](#metadata-topic)
     * [Post-Processing Filters](#post-processing-filters)
     * [Available Services](#available-services)
     * [Efficient intra-process communication](#efficient-intra-process-communication)
  * [Contributing](CONTRIBUTING.md)
  * [License](LICENSE)

<hr>

# ROS1 and ROS2 Legacy

<details>
  <summary>
    Intel RealSense ROS1 Wrapper
  </summary>
    Intel Realsense ROS1 Wrapper is not supported anymore, since our developers team are focusing on ROS2 distro.<br>
    For ROS1 wrapper, go to <a href="https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy">ros1-legacy</a> branch
</details>

<details>
   <summary>
     Moving from <a href="https://github.com/IntelRealSense/realsense-ros/tree/ros2-legacy">ros2-legacy</a> to ros2-development
  </summary>

* Changed Parameters:
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

</details>
    

# Installation
  
<details>
  <summary>
    Step 1: Install the ROS2 distribution 
  </summary>
  
- #### Ubuntu 22.04:
  - [ROS2 Iron](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)
  - [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
    
</details>
  
<details>
  <summary>
    Step 2: Install latest Intel&reg; RealSense&trade; SDK 2.0
  </summary>
  
- #### Option 1: Install librealsense2 debian package from Intel servers
  - Jetson users - use the [Jetson Installation Guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md)
  - Otherwise, install from [Linux Debian Installation Guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)
    - In this case treat yourself as a developer: make sure to follow the instructions to also install librealsense2-dev and librealsense2-dkms packages
  
- #### Option 2: Install librealsense2 (without graphical tools and examples) debian package from ROS servers:
  - [Configure](http://wiki.ros.org/Installation/Ubuntu/Sources) your Ubuntu repositories
  - Install all realsense ROS packages by ```sudo apt install ros-<ROS_DISTRO>-librealsense2*```
    - For example, for Humble distro: ```sudo apt install ros-humble-librealsense2*```

- #### Option 3: Build from source
  - Download the latest [Intel&reg; RealSense&trade; SDK 2.0](https://github.com/IntelRealSense/librealsense/releases/tag/v2.53.1)
  - Follow the instructions under [Linux Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

</details>
  
<details>
  <summary>
    Step 3: Install Intel&reg; RealSense&trade; ROS2 wrapper
  </summary>
  
#### Option 1: Install debian package from ROS servers
  - [Configure](http://wiki.ros.org/Installation/Ubuntu/Sources) your Ubuntu repositories
  - Install all realsense ROS packages by ```sudo apt install ros-<ROS_DISTRO>-realsense2-*```
  - For example, for Humble distro: ```sudo apt install ros-humble-realsense2-*```
  
#### Option 2: Install from source
  
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
  
  - Install dependencies
   ```bash
   sudo apt-get install python3-rosdep -y
   sudo rosdep init # "sudo rosdep init --include-eol-distros" for Foxy and earlier
   rosdep update # "sudo rosdep update --include-eol-distros" for Foxy and earlier
   rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
   ```

  - Build
   ```bash
   colcon build
   ```

  -  Source environment
   ```bash
   ROS_DISTRO=<YOUR_SYSTEM_ROS_DISTRO>  # set your ROS_DISTRO: iron, humble
   source /opt/ros/$ROS_DISTRO/setup.bash
   cd ~/ros2_ws
   . install/local_setup.bash
   ```
  
  </details>

<hr>

# Usage

## Start the camera node
  
  #### with ros2 run:
    ros2 run realsense2_camera realsense2_camera_node
    # or, with parameters, for example - temporal and spatial filters are enabled:
    ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_color:=false -p spatial_filter.enable:=true -p temporal_filter.enable:=true
  
  #### with ros2 launch:
    ros2 launch realsense2_camera rs_launch.py
    ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=true

<hr>

## Camera Name And Camera Namespace

User can set the camera name and camera namespace, to distinguish between cameras and platforms, which helps identifying the right nodes and topics to work with.

### Example
- If user have multiple cameras (might be of the same model) and multiple robots then user can choose to launch/run his nodes on this way.
- For the first robot and first camera he will run/launch it with these parameters:
  - camera_namespace:
    - robot1
  - camera_name
    - D455_1
  
  - With ros2 launch (via command line or by editing these two parameters in the launch file):
    
  ```ros2 launch realsense2_camera rs_launch.py camera_namespace:=robot1 camera_name:=D455_1```
    
  - With ros2 run (using remapping mechanisim [Reference](https://docs.ros.org/en/humble/How-To-Guides/Node-arguments.html)):
    
  ```ros2 run realsense2_camera realsense2_camera_node --ros-args -r __node:=D455_1 -r __ns:=robot1```

  - Result
  ```
  > ros2 node list
  /robot1/D455_1
  
  > ros2 topic list
  /robot1/D455_1/color/camera_info
  /robot1/D455_1/color/image_raw
  /robot1/D455_1/color/metadata
  /robot1/D455_1/depth/camera_info
  /robot1/D455_1/depth/image_rect_raw
  /robot1/D455_1/depth/metadata
  /robot1/D455_1/extrinsics/depth_to_color
  /robot1/D455_1/imu
  
  > ros2 service list
  /robot1/D455_1/device_info
  ```

### Default behavior if non of these parameters are given:
  - camera_namespace:=camera
  - camera_name:=camera

```
> ros2 node list
/camera/camera

> ros2 topic list
/camera/camera/color/camera_info
/camera/camera/color/image_raw
/camera/camera/color/metadata
/camera/camera/depth/camera_info
/camera/camera/depth/image_rect_raw
/camera/camera/depth/metadata
/camera/camera/extrinsics/depth_to_color
/camera/camera/imu

> ros2 service list
/camera/camera/device_info
```

<hr>

## Parameters

### Available Parameters:
- For the entire list of parameters type `ros2 param list`.
- For reading a parameter value use `ros2 param get <node> <parameter_name>` 
  - For example: `ros2 param get /camera/camera depth_module.emitter_enabled`
- For setting a new value for a parameter use `ros2 param set <node> <parameter_name> <value>`
  - For example: `ros2 param set /camera/camera depth_module.emitter_enabled 1`

#### Parameters that can be modified during runtime:
- All of the filters and sensors inner parameters.
- Video Sensor Parameters: (```depth_module``` and ```rgb_camera```)
  - They have, at least, the **profile** parameter.
    - The profile parameter is a string of the following format: \<width>X\<height>X\<fps> (The dividing character can be X, x or ",". Spaces are ignored.)
    - For example: ```depth_module.profile:=640x480x30 rgb_camera.profile:=1280x720x30```
    - Since infra, infra1, infra2 and depth are all streams of the depth_module, their width, height and fps are defined by the same param **depth_module.profile**
    - If the specified combination of parameters is not available by the device, the default or previously set configuration will be used.
      - Run ```ros2 param describe <your_node_name> <param_name>``` to get the list of supported profiles.
    - Note: Should re-enable the stream for the change to take effect.
  - ***<stream_name>*_format**
    - This parameter is a string used to select the stream format.
    - <stream_name> can be any of *infra, infra1, infra2, color, depth*.
    - For example: ```depth_module.depth_format:=Z16 depth_module.infra1_format:=y8 rgb_camera.color_format:=RGB8```
    - This parameter supports both lower case and upper case letters.
    - If the specified parameter is not available by the stream, the default or previously set configuration will be used.
      - Run ```ros2 param describe <your_node_name> <param_name>``` to get the list of supported formats.
    - Note: Should re-enable the stream for the change to take effect.
  - If the stream doesn't support the user selected profile \<width>X\<height>X\<fps> + \<format>, it will not be opened and a warning message will be shown.
    - Should update the profile settings and re-enable the stream for the change to take effect.
    - Run ```rs-enumerate-devices``` command to know the list of profiles supported by the connected sensors.
- **enable_*<stream_name>***: 
  - Choose whether to enable a specified stream or not. Default is true for images and false for orientation streams.
  - <stream_name> can be any of *infra, infra1, infra2, color, depth, gyro, accel*.
  - For example: ```enable_infra1:=true enable_color:=false```
- **enable_sync**:
  - gathers closest frames of different sensors, infra red, color and depth, to be sent with the same timetag.
  - This happens automatically when such filters as pointcloud are enabled.
- ***<stream_type>*_qos**: 
  - Sets the QoS by which the topic is published.
  - <stream_type> can be any of *infra, infra1, infra2, color, depth, gyro, accel*.
  -  Available values are the following strings: `SYSTEM_DEFAULT`, `DEFAULT`, `PARAMETER_EVENTS`, `SERVICES_DEFAULT`, `PARAMETERS`, `SENSOR_DATA`.
  - For example: ```depth_qos:=SENSOR_DATA```
  - Reference: [ROS2 QoS profiles formal documentation](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html#qos-profiles)
- **Notice:** ***<stream_type>*_info_qos** refers to both camera_info topics and metadata topics.
- **tf_publish_rate**: 
  - double, rate (in Hz) at which dynamic transforms are published
  - Default value is 0.0 Hz *(means no dynamic TF)*
  - This param also depends on **publish_tf** param
    - If **publish_tf:=false**, then no TFs will be published, even if **tf_publish_rate** is >0.0 Hz
    - If **publish_tf:=true** and **tf_publish_rate** set to >0.0 Hz, then dynamic TFs will be published at the specified rate
- **unite_imu_method**:
  - For the D400 cameras with built in IMU components, below 2 unrelated streams (each with it's own frequency) will be created:
    - *gyro* - which shows angular velocity 
    - *accel* - which shows linear acceleration. 
  - Both streams will publish data to its corresponding topics:
    - '/camera/camera/gyro/sample' & '/camera/camera/accel/sample'
    - Though both topics are of same message type 'sensor_msgs::Imu', only their relevant fields are filled out.
  - A new topic called **imu** will be created, when both *accel* and *gyro* streams are enabled and the param *unite_imu_method* set to > 0.
    - Data from both accel and gyro are combined and published to this topic
    - All the fields of the Imu message are filled out.
    - It will be published at the rate of the gyro.
  - `unite_imu_method` param supports below values:
    - 0 -> **none**: no imu topic
    - 1 -> **copy**: Every gyro message will be attached by the last accel message.
    - 2 -> **linear_interpolation**: Every gyro message will be attached by an accel message which is interpolated to gyro's timestamp.
  - Note: When the param *unite_imu_method* is dynamically updated, re-enable either gyro or accel stream for the change to take effect.

#### Parameters that cannot be changed in runtime:
- **serial_no**:
  - will attach to the device with the given serial number (*serial_no*) number.
  - Default, attach to the first (in an inner list) RealSense device.
  - Note: serial number should be defined with "_" prefix.
    - That is a workaround until a better method will be found to ROS2's auto conversion of strings containing only digits into integers.
  - Example: serial number 831612073525 can be set in command line as `serial_no:=_831612073525`. 
- **usb_port_id**:
  - will attach to the device with the given USB port (*usb_port_id*).
  - For example: `usb_port_id:=4-1` or `usb_port_id:=4-2` 
  - Default, ignore USB port when choosing a device.
- **device_type**:
  - will attach to a device whose name includes the given *device_type* regular expression pattern.
  - Default, ignore device type.
  - For example:
    - `device_type:=d435` will match d435 and d435i.
    - `device_type=d435(?!i)` will match d435 but not d435i.
- **reconnect_timeout**:
  - When the driver cannot connect to the device try to reconnect after this timeout (in seconds).
  - For Example: `reconnect_timeout:=10`
- **wait_for_device_timeout**: 
  - If the specified device is not found, will wait *wait_for_device_timeout* seconds before exits.
  - Defualt, *wait_for_device_timeout < 0*, will wait indefinitely.
  - For example: `wait_for_device_timeout:=60`
- **rosbag_filename**:
  - Publish topics from rosbag file. There are two ways for loading rosbag file:
   * Command line - ```ros2 run realsense2_camera realsense2_camera_node -p rosbag_filename:="/full/path/to/rosbag.bag"```
   * Launch file - set ```rosbag_filename``` parameter with rosbag full path (see ```realsense2_camera/launch/rs_launch.py``` as reference) 
- **initial_reset**:
  - On occasions the device was not closed properly and due to firmware issues needs to reset. 
  - If set to true, the device will reset prior to usage.
  - For example: `initial_reset:=true`
- **base_frame_id**: defines the frame_id all static transformations refers to.
- **clip_distance**:
  - Remove from the depth image all values above a given value (meters). Disable by giving negative value (default)
  - For example: `clip_distance:=1.5`
- **linear_accel_cov**, **angular_velocity_cov**: sets the variance given to the Imu readings.
- **hold_back_imu_for_frames**: Images processing takes time. Therefor there is a time gap between the moment the image arrives at the wrapper and the moment the image is published to the ROS environment. During this time, Imu messages keep on arriving and a situation is created where an image with earlier timestamp is published after Imu message with later timestamp. If that is a problem, setting *hold_back_imu_for_frames* to *true* will hold the Imu messages back while processing the images and then publish them all in a burst, thus keeping the order of publication as the order of arrival. Note that in either case, the timestamp in each message's header reflects the time of it's origin.
- **publish_tf**:
  - boolean, enable/disable publishing static and dynamic TFs
  - Defaults to True
    - So, static TFs will be published by default
    - If dynamic TFs are needed, user should set the param **tf_publish_rate** to >0.0 Hz
  - If set to false, both static and dynamic TFs won't be published, even if the param **tf_publish_rate** is set to >0.0 Hz
- **diagnostics_period**: 
  - double, positive values set the period between diagnostics updates on the `/diagnostics` topic.
  - 0 or negative values mean no diagnostics topic is published. Defaults to 0.</br>
The `/diagnostics` topic includes information regarding the device temperatures and actual frequency of the enabled streams.

<hr>

## ROS2(Robot) vs Optical(Camera) Coordination Systems:

- Point Of View:
  - Imagine we are standing behind of the camera, and looking forward.
  - Always use this point of view when talking about coordinates, left vs right IRs, position of sensor, etc..

![image](https://user-images.githubusercontent.com/99127997/230150735-bc31fedf-d715-4e35-b462-fe2c338832c3.png)

- ROS2 Coordinate System: (X: Forward, Y:Left, Z: Up)
- Camera Optical Coordinate System: (X: Right, Y: Down, Z: Forward)
- References: [REP-0103](https://www.ros.org/reps/rep-0103.html#coordinate-frame-conventions), [REP-0105](https://www.ros.org/reps/rep-0105.html#coordinate-frames)
- All data published in our wrapper topics is optical data taken directly from our camera sensors.
- static and dynamic TF topics publish optical CS and ROS CS to give the user the ability to move from one CS to other CS.

<hr>

## TF from coordinate A to coordinate B:

- TF msg expresses a transform from coordinate frame "header.frame_id" (source) to the coordinate frame child_frame_id (destination) [Reference](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Transform.html)
- In RealSense cameras, the origin point (0,0,0) is taken from the left IR (infra1) position and named as "camera_link" frame
- Depth, left IR and "camera_link" coordinates converge together.   
- Our wrapper provide static TFs between each sensor coordinate to the camera base (camera_link)
- Also, it provides TFs from each sensor ROS coordinates to its corrosponding optical coordinates.
- Example of static TFs of RGB sensor and Infra2 (right infra) sensor of D435i module as it shown in rviz2: 
![example](https://user-images.githubusercontent.com/99127997/230148106-0f79cbdb-c401-4d09-b386-a366af18e5f7.png)

<hr>

## Extrinsics from sensor A to sensor B:

- Extrinsic from sensor A to sensor B means the position and orientation of sensor A relative to sensor B.
- Imagine that B is the origin (0,0,0), then the Extrensics(A->B) describes where is sensor A relative to sensor B.
- For example, depth_to_color, in D435i:
  - If we look from behind of the D435i, extrinsic from depth to color, means, where is the depth in relative to the color.
  - If we just look at the X coordinates, in the optical coordiantes (again, from behind) and assume that COLOR(RGB) sensor is (0,0,0), we can say that DEPTH sensor is on the right of RGB by 0.0148m (1.48cm).

![d435i](https://user-images.githubusercontent.com/99127997/230220297-e392f0fc-63bf-4bab-8001-af1ddf0ed00e.png)

```
administrator@perclnx466 ~/ros2_humble $ ros2 topic echo /camera/camera/extrinsics/depth_to_color
rotation:
- 0.9999583959579468
- 0.008895332925021648
- -0.0020127370953559875
- -0.008895229548215866
- 0.9999604225158691
- 6.045500049367547e-05
- 0.0020131953060626984
- -4.254872692399658e-05
- 0.9999979734420776
translation:
- 0.01485931035131216
- 0.0010161789832636714
- 0.0005317096947692335
---
```

- Extrinsic msg is made up of two parts:
  - float64[9] rotation  (Column - major 3x3 rotation matrix)
  - float64[3] translation  (Three-element translation vector, in meters)

<hr>

## Published Topics
  
The published topics differ according to the device and parameters.
After running the above command with D435i attached, the following list of topics will be available (This is a partial list. For full one type `ros2 topic list`):
- /camera/camera/aligned_depth_to_color/camera_info
- /camera/camera/aligned_depth_to_color/image_raw
- /camera/camera/color/camera_info
- /camera/camera/color/image_raw
- /camera/camera/color/metadata
- /camera/camera/depth/camera_info
- /camera/camera/depth/color/points
- /camera/camera/depth/image_rect_raw
- /camera/camera/depth/metadata
- /camera/camera/extrinsics/depth_to_color
- /camera/camera/imu
- /diagnostics
- /parameter_events
- /rosout
- /tf_static

This will stream relevant camera sensors and publish on the appropriate ROS topics.

  Enabling accel and gyro is achieved either by adding the following parameters to the command line:</br>
`ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true enable_gyro:=true enable_accel:=true` </br>
or in runtime using the following commands:
```
ros2 param set /camera/camera enable_accel true
ros2 param set /camera/camera enable_gyro true
```

Enabling stream adds matching topics. For instance, enabling the gyro and accel streams adds the following topics:
- /camera/camera/accel/imu_info
- /camera/camera/accel/metadata
- /camera/camera/accel/sample
- /camera/camera/extrinsics/depth_to_accel
- /camera/camera/extrinsics/depth_to_gyro
- /camera/camera/gyro/imu_info
- /camera/camera/gyro/metadata
- /camera/camera/gyro/sample

<hr>

## RGBD Topic

RGBD new topic, publishing [RGB + Depth] in the same message (see RGBD.msg for reference). For now, works only with depth aligned to color images, as color and depth images are synchronized by frame time tag.

These boolean paramters should be true to enable rgbd messages:

- `enable_rgbd`: new paramter, to enable/disable rgbd topic, changeable during runtime
- `align_depth.enable`: align depth images to rgb images
- `enable_sync`: let librealsense sync between frames, and get the frameset with color and depth images combined
- `enable_color` + `enable_depth`: enable both color and depth sensors

The current QoS of the topic itself, is the same as Depth and Color streams (SYSTEM_DEFAULT)

Example:
```
ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true 
```

<hr>

## Metadata topic
  
The metadata messages store the camera's available metadata in a *json* format. To learn more, a dedicated script for echoing a metadata topic in runtime is attached. For instance, use the following command to echo the camera/depth/metadata topic:
```
python3 src/realsense-ros/realsense2_camera/scripts/echo_metadada.py /camera/camera/depth/metadata
```
  
<hr>

## Post-Processing Filters

The following post processing filters are available:
 - ```align_depth```: If enabled, will publish the depth image aligned to the color image on the topic `/camera/camera/aligned_depth_to_color/image_raw`.
   - The pointcloud, if created, will be based on the aligned depth image.
 - ```colorizer```: will color the depth image. On the depth topic an RGB image will be published, instead of the 16bit depth values .
 - ```pointcloud```: will add a pointcloud topic `/camera/camera/depth/color/points`.
    * The texture of the pointcloud can be modified using the `pointcloud.stream_filter` parameter.</br>
    * The depth FOV and the texture FOV are not similar. By default, pointcloud is limited to the section of depth containing the texture. You can have a full depth to pointcloud, coloring the regions beyond the texture with zeros, by setting `pointcloud.allow_no_texture_points` to true.
    * pointcloud is of an unordered format by default. This can be changed by setting `pointcloud.ordered_pc` to true.
 - ```hdr_merge```: Allows depth image to be created by merging the information from 2 consecutive frames, taken with different exposure and gain values.
  - `depth_module.hdr_enabled`: to enable/disable HDR
  - The way to set exposure and gain values for each sequence in runtime is by first selecting the sequence id, using the `depth_module.sequence_id` parameter and then modifying the `depth_module.gain`, and `depth_module.exposure`.
    - From FW versions 5.14.x.x and above, if HDR is enabled, the preset configs (like exposure, gain, etc.,) cannot be updated.
      - Disable the HDR first using `depth_module.hdr_enabled` parameter and then, update the required presets.
  - To view the effect on the infrared image for each sequence id use the `filter_by_sequence_id.sequence_id` parameter.
  - To initialize these parameters in start time use the following parameters:
    - `depth_module.exposure.1`
    - `depth_module.gain.1`
    - `depth_module.exposure.2`
    - `depth_module.gain.2`
  - For in-depth review of the subject please read the accompanying [white paper](https://dev.intelrealsense.com/docs/high-dynamic-range-with-stereoscopic-depth-cameras).

  - The following filters have detailed descriptions in : https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md
    - ```disparity_filter``` - convert depth to disparity before applying other filters and back.
    - ```spatial_filter``` - filter the depth image spatially.
    - ```temporal_filter``` - filter the depth image temporally.
    - ```hole_filling_filter``` - apply hole-filling filter.
    - ```decimation_filter``` - reduces depth scene complexity.

Each of the above filters have it's own parameters, following the naming convention of `<filter_name>.<parameter_name>` including a `<filter_name>.enable` parameter to enable/disable it. 

<hr>

## Available services
  
- device_info : retrieve information about the device - serial_number, firmware_version etc. Type `ros2 interface show realsense2_camera_msgs/srv/DeviceInfo` for the full list. Call example: `ros2 service call /camera/camera/device_info realsense2_camera_msgs/srv/DeviceInfo`

<hr>

## Efficient intra-process communication:
  
Our ROS2 Wrapper node supports zero-copy communications if loaded in the same process as a subscriber node. This can reduce copy times on image/pointcloud topics, especially with big frame resolutions and high FPS.

You will need to launch a component container and launch our node as a component together with other component nodes. Further details on "Composing multiple nodes in a single process" can be found [here](https://docs.ros.org/en/rolling/Tutorials/Composition.html).

Further details on efficient intra-process communication can be found [here](https://docs.ros.org/en/humble/Tutorials/Intra-Process-Communication.html#efficient-intra-process-communication).

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

</details>


[rolling-badge]: https://img.shields.io/badge/-ROLLING-orange?style=flat-square&logo=ros
[rolling]: https://docs.ros.org/en/rolling/index.html
[humble-badge]: https://img.shields.io/badge/-HUMBLE-orange?style=flat-square&logo=ros
[humble]: https://docs.ros.org/en/humble/index.html
[iron-badge]: https://img.shields.io/badge/-IRON-orange?style=flat-square&logo=ros
[iron]: https://docs.ros.org/en/iron/index.html
[ubuntu22-badge]: https://img.shields.io/badge/-UBUNTU%2022%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu22]: https://releases.ubuntu.com/jammy/
[ubuntu20-badge]: https://img.shields.io/badge/-UBUNTU%2020%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu20]: https://releases.ubuntu.com/focal/
