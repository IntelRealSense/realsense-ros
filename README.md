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
  * [ROS1 and ROS2 legacy](#legacy)
  * [Installation](#installation)
  * [Usage](#usage)
     * [Starting the camera node](#start-camera-node)
     * [Parameters](#parameters)
     * [ROS2-vs-Optical Coordination Systems](#coordination)
     * [TF from coordinate A to coordinate B](#tfs)
     * [Extrinsics from sensor A to sensor B](#extrinsics)
     * [Topics](#topics)
     * [Metadata Topic](#metadata)
     * [Post-Processing Filters](#filters)
     * [Available Services](#services)
     * [Efficient intra-process communication](#intra-process)
  * [Contributing](CONTRIBUTING.md)
  * [License](LICENSE)

<hr>

<h2 id="legacy">
  Legacy
</h2>

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
    

<h2 id="installation">
  Installation
</h2>
  
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
   sudo rosdep init # "sudo rosdep init --include-eol-distros" for Eloquent and earlier
   rosdep update # "sudo rosdep update --include-eol-distros" for Eloquent and earlier
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

<h2 id="usage">
  Usage
</h2>

<h3 id="start-camera-node">
  Start the camera node
</h3>
  
  #### with ros2 run:
    ros2 run realsense2_camera realsense2_camera_node
    # or, with parameters, for example - temporal and spatial filters are enabled:
    ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_color:=false -p spatial_filter.enable:=true -p temporal_filter.enable:=true
  
  #### with ros2 launch:
    ros2 launch realsense2_camera rs_launch.py
    ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=true

<hr>

<h3 id="parameters">
  Parameters
<h3>
  
### Sensor Parameters:
- Each sensor has a unique set of parameters.
- Video sensors, such as depth_module or rgb_camera have, at least, the 'profile' parameter.</br>
  - The profile parameter is a string of the following format: \<width>X\<height>X\<fps> (The deviding character can be X, x or ",". Spaces are ignored.)
  - For example: ```depth_module.profile:=640x480x30```
- Since infra1, infra2 and depth are all streams of the depth_module, their width, height and fps are defined by their common sensor.
- If the specified combination of parameters is not available by the device, the default configuration will be used.

</br>

### Available Parameters:
- For the entire list of parameters type `ros2 param list`.
- For reading a parameter value use `ros2 param get <node> <parameter_name>` 
  - For example: `ros2 param get /camera/camera depth_module.emitter_on_off`
- For setting a new value for a parameter use `ros2 param set <node> <parameter_name> <value>`
  - For example: `ros2 param set /camera/camera depth_module.emitter_on_off true`

#### Parameters that can be modified during runtime:
- All of the filters and sensors inner parameters.
- **enable_*<stream_name>***: 
  - Choose whether to enable a specified stream or not. Default is true for images and false for orientation streams.
  - <stream_name> can be any of *infra1, infra2, color, depth, fisheye, fisheye1, fisheye2, gyro, accel, pose*.
  - For example: ```enable_infra1:=true enable_color:=false```
- **enable_sync**:
  - gathers closest frames of different sensors, infra red, color and depth, to be sent with the same timetag.
  - This happens automatically when such filters as pointcloud are enabled.
- ***<stream_type>*_qos**: 
  - Sets the QoS by which the topic is published.
  - <stream_type> can be any of *infra, color, fisheye, depth, gyro, accel, pose*.
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
- ***<stream_name>*_frame_id**, ***<stream_name>*_optical_frame_id**, **aligned_depth_to_*<stream_name>*_frame_id**: Specify the different frame_id for the different frames. Especially important when using multiple cameras.
- **base_frame_id**: defines the frame_id all static transformations refers to.
- **odom_frame_id**: defines the origin coordinate system in ROS convention (X-Forward, Y-Left, Z-Up). pose topic defines the pose relative to that system.

- **unite_imu_method**:
  - D400 cameras have built in IMU components which produce 2 unrelated streams, each with it's own frequency: 
    - *gyro* - which shows angular velocity 
    - *accel* which shows linear acceleration. 
  - By default, 2 corresponding topics are available, each with only the relevant fields of the message sensor_msgs::Imu are filled out.
  - Setting *unite_imu_method* creates a new topic, *imu*, that replaces the default *gyro* and *accel* topics.
    - The *imu* topic is published at the rate of the gyro.
    - All the fields of the Imu message under the *imu* topic are filled out. 
  - `unite_imu_method` parameter supported values are [0-2] meaning:  [0 -> None, 1 -> Copy, 2 -> Linear_ interpolation] when:
    - **linear_interpolation**: Every gyro message is attached by the an accel message interpolated to the gyro's timestamp.
    - **copy**: Every gyro message is attached by the last accel message.
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

- **publish_odom_tf**: If True (default) publish TF from odom_frame to pose_frame.

<hr>

<h3 id="coordination">
  ROS2(Robot) vs Optical(Camera) Coordination Systems:
</h3>

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

<h3 id="tfs">
   TF from coordinate A to coordinate B:
</h3>

- TF msg expresses a transform from coordinate frame "header.frame_id" (source) to the coordinate frame child_frame_id (destination) [Reference](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Transform.html)
- In RealSense cameras, the origin point (0,0,0) is taken from the left IR (infra1) position and named as "camera_link" frame
- Depth, left IR and "camera_link" coordinates converge together.   
- Our wrapper provide static TFs between each sensor coordinate to the camera base (camera_link)
- Also, it provides TFs from each sensor ROS coordinates to its corrosponding optical coordinates.
- Example of static TFs of RGB sensor and Infra2 (right infra) sensor of D435i module as it shown in rviz2: 
![example](https://user-images.githubusercontent.com/99127997/230148106-0f79cbdb-c401-4d09-b386-a366af18e5f7.png)

<hr>

<h3 id="extrinsics">
   Extrinsics from sensor A to sensor B:
</h3>


- Extrinsic from sensor A to sensor B means the position and orientation of sensor A relative to sensor B.
- Imagine that B is the origin (0,0,0), then the Extrensics(A->B) describes where is sensor A relative to sensor B.
- For example, depth_to_color, in D435i:
  - If we look from behind of the D435i, extrinsic from depth to color, means, where is the depth in relative to the color.
  - If we just look at the X coordinates, in the optical coordiantes (again, from behind) and assume that COLOR(RGB) sensor is (0,0,0), we can say that DEPTH sensor is on the right of RGB by 0.0148m (1.48cm).

![d435i](https://user-images.githubusercontent.com/99127997/230220297-e392f0fc-63bf-4bab-8001-af1ddf0ed00e.png)

```
administrator@perclnx466 ~/ros2_humble $ ros2 topic echo /camera/extrinsics/depth_to_color
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

<h3 id="topics">
  Published Topics
</h3>
  
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

This will stream relevant camera sensors and publish on the appropriate ROS topics.

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

<hr>

<h3 id="metadata">
  Metadata topic
</h3>
  
The metadata messages store the camera's available metadata in a *json* format. To learn more, a dedicated script for echoing a metadata topic in runtime is attached. For instance, use the following command to echo the camera/depth/metadata topic:
```
python3 src/realsense-ros/realsense2_camera/scripts/echo_metadada.py /camera/depth/metadata
```
  
<hr>

<h3 id="filters">
  Post-Processing Filters
</h3>
  
The following post processing filters are available:
 - ```align_depth```: If enabled, will publish the depth image aligned to the color image on the topic `/camera/aligned_depth_to_color/image_raw`.
   - The pointcloud, if created, will be based on the aligned depth image.
 - ```colorizer```: will color the depth image. On the depth topic an RGB image will be published, instead of the 16bit depth values .
 - ```pointcloud```: will add a pointcloud topic `/camera/depth/color/points`.
    * The texture of the pointcloud can be modified using the `pointcloud.stream_filter` parameter.</br>
    * The depth FOV and the texture FOV are not similar. By default, pointcloud is limited to the section of depth containing the texture. You can have a full depth to pointcloud, coloring the regions beyond the texture with zeros, by setting `pointcloud.allow_no_texture_points` to true.
    * pointcloud is of an unordered format by default. This can be changed by setting `pointcloud.ordered_pc` to true.
 - ```hdr_merge```: Allows depth image to be created by merging the information from 2 consecutive frames, taken with different exposure and gain values.
  - The way to set exposure and gain values for each sequence in runtime is by first selecting the sequence id, using the `depth_module.sequence_id` parameter and then modifying the `depth_module.gain`, and `depth_module.exposure`.
  - To view the effect on the infrared image for each sequence id use the `sequence_id_filter.sequence_id` parameter.
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

<h3 id="services">
  Available services
</h3>
  
- device_info : retrieve information about the device - serial_number, firmware_version etc. Type `ros2 interface show realsense2_camera_msgs/srv/DeviceInfo` for the full list. Call example: `ros2 service call /camera/device_info realsense2_camera_msgs/srv/DeviceInfo`

<hr>

<h3 id="intra-process">
  Efficient intra-process communication:
</h3>
  
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
