<h1 align="center">
   <img src="https://www.intelrealsense.com/wp-content/uploads/2020/09/intel-realsense-logo-360px.png" alt="Intel® RealSense™" title="Intel® RealSense™" />
</h1>

<p align="center">
  ROS2 packages for using Intel RealSense D400 cameras.<br>
  Supported ROS2 Distros: Dashing, Eloquent, Foxy, Humble and Rolling.<br>
  <a href="https://github.com/IntelRealSense/realsense-ros/releases">Latest release notes</a>
</p>

<hr>

## Table of contents
  * [ROS1 and ROS2 legacy](#legacy)
  * [Installation](#installation)
  * [Usage](#usage)
     * [Starting the camera node](#start-camera-node)
     * [Parameters](#parameters)
     * [Topics](#topics)
     * [Metadata Topic](#metadata)
     * [Post-Processing Filters](#filters)
     * [Available Services](#services)
     * [Efficient intra-process communication](#intra-process)
  * [Contributing](#contributing)
  * [License](#license)

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
  - [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- #### Ubuntu 20.04: 
  - [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
  - [ROS2 Galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
- #### Ubuntu 18.04 : 
  - [ROS2 Dashing](https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Debians.html)
  - [ROS2 Eloquent](https://docs.ros.org/en/eloquent/Installation/Linux-Install-Debians.html)
    
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
   ROS_DISTRO=<YOUR_SYSTEM_ROS_DISTRO>  # set your ROS_DISTRO: humble, galactic, foxy, eloquent, dashing
   source /opt/ros/$ROS_DISTRO/setup.bash
   cd ~/ros2_ws
   . install/local_setup.bash
   ```
  
  </details>

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
  
<h3 id="parameters">
  Parameters
<h3>
  
### Sensor Parameters:
Each sensor has a unique set of parameters.
Video sensors, such as depth_module or rgb_camera have, at least, the 'profile' parameter.</br>
It is a string of the following format: \<width>X\<height>X\<fps> (The deviding character can be X, x or ",". Spaces are ignored.)

Since infra1, infra2 and depth are all streams of the depth_module, their width, height and fps are defined by their common sensor.
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

<h3 id="metadata">
  Metadata topic
</h3>
  
The metadata messages store the camera's available metadata in a *json* format. To learn more, a dedicated script for echoing a metadata topic in runtime is attached. For instance, use the following command to echo the camera/depth/metadata topic:
```
python3 src/realsense-ros/realsense2_camera/scripts/echo_metadada.py /camera/depth/metadata
```
  
<h3 id="filters">
  Post-Processing Filters
</h3>
  
The following post processing filters are available:
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

<h3 id="services">
  Available services
</h3>
  
- device_info : retrieve information about the device - serial_number, firmware_version etc. Type `ros2 interface show realsense2_camera_msgs/srv/DeviceInfo` for the full list. Call example: `ros2 service call /camera/device_info realsense2_camera_msgs/srv/DeviceInfo`
  - Note that for **ROS2 Dashing** the command is `ros2 srv show realsense2_camera_msgs/srv/DeviceInfo`

<h3 id="intra-process">
  Efficient intra-process communication:
</h3>
  
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

  <details>
    <summary>
      <h3 id="contributing">
        Contributing
      </h3>
    </summary>
  # How to Contribute

This project welcomes third-party code via GitHub pull requests.

You are welcome to propose and discuss enhancements using project [issues](https://github.com/IntelRealSense/realsense-ros/issues).

> **Branching Policy**:
> The `ros2-development` branch is considered stable, at all times.
> If you plan to propose a patch, please commit into the `ros2-development` branch, or its own feature branch.

In addition, please run `pr_check.sh` under `scripts` directory. This scripts verify compliance with project's standards:

1. Every example / source file must refer to [LICENSE](https://github.com/IntelRealSense/realsense-ros/blob/ros2-development/LICENSE)
2. Every example / source file must include correct copyright notice
3. For indentation we are using spaces and not tabs
4. Line-endings must be Unix and not DOS style

Most common issues can be automatically resolved by running `./pr_check.sh --fix`

Please familirize yourself with the [Apache License 2.0](https://github.com/IntelRealSense/realsense-ros/blob/ros2-development/LICENSE) before contributing.

## Step-by-Step

1. Make sure you have `git` and `cmake` installed on your system. On Windows we recommend using [Git Extensions](https://github.com/gitextensions/gitextensions/releases) for git bash.
2. Run `git clone https://github.com/IntelRealSense/realsense-ros.git` and `cd realsense-ros`
3. To align with latest status of the ros2-development branch, run:
```
git fetch origin
git checkout ros2-development
git reset --hard origin/ros2-development
```
4. `git checkout -b name_of_your_contribution` to create a dedicated branch
5. Make your changes to the local repository
6. Make sure your local git user is updated, or run `git config --global user.email "email@example.com"` and `git config --global user.user "user"` to set it up. This is the user & email that will appear in GitHub history.
7. `git add -p` to select the changes you wish to add
8. `git commit -m "Description of the change"`
9. Make sure you have a GitHub user and [fork realsense-ros](https://github.com/IntelRealSense/realsense-ros#fork-destination-box)
10. `git remote add fork https://github.com/username/realsense-ros.git` with your GitHub `username`
11. `git fetch fork`
12. `git push fork` to push `name_of_your_contribution` branch to your fork
13. Go to your fork on GitHub at `https://github.com/username/realsense-ros`
14. Click the `New pull request` button
15. For `base` combo-box select `ros2-development`, since you want to submit a PR to that branch
16. For `compare` combo-box select `name_of_your_contribution` with your commit
17. Review your changes and click `Create pull request`
18. Wait for all automated checks to pass
19. The PR will be approved / rejected after review from the team and the community

To continue to new change, goto step 3.
To return to your PR (in order to make more changes):
1. `git stash`
2. `git checkout name_of_your_contribution`
3. Repeat items 5-8 from the previous list
4. `git push fork`
The pull request will be automatically updated

</details>

<details>
  <summary>
    <h3 id="license">
      License
    </h3>
  </summary>
  
                                 Apache License
                           Version 2.0, January 2004
                        http://www.apache.org/licenses/

   TERMS AND CONDITIONS FOR USE, REPRODUCTION, AND DISTRIBUTION

   1. Definitions.

      "License" shall mean the terms and conditions for use, reproduction,
      and distribution as defined by Sections 1 through 9 of this document.

      "Licensor" shall mean the copyright owner or entity authorized by
      the copyright owner that is granting the License.

      "Legal Entity" shall mean the union of the acting entity and all
      other entities that control, are controlled by, or are under common
      control with that entity. For the purposes of this definition,
      "control" means (i) the power, direct or indirect, to cause the
      direction or management of such entity, whether by contract or
      otherwise, or (ii) ownership of fifty percent (50%) or more of the
      outstanding shares, or (iii) beneficial ownership of such entity.

      "You" (or "Your") shall mean an individual or Legal Entity
      exercising permissions granted by this License.

      "Source" form shall mean the preferred form for making modifications,
      including but not limited to software source code, documentation
      source, and configuration files.

      "Object" form shall mean any form resulting from mechanical
      transformation or translation of a Source form, including but
      not limited to compiled object code, generated documentation,
      and conversions to other media types.

      "Work" shall mean the work of authorship, whether in Source or
      Object form, made available under the License, as indicated by a
      copyright notice that is included in or attached to the work
      (an example is provided in the Appendix below).

      "Derivative Works" shall mean any work, whether in Source or Object
      form, that is based on (or derived from) the Work and for which the
      editorial revisions, annotations, elaborations, or other modifications
      represent, as a whole, an original work of authorship. For the purposes
      of this License, Derivative Works shall not include works that remain
      separable from, or merely link (or bind by name) to the interfaces of,
      the Work and Derivative Works thereof.

      "Contribution" shall mean any work of authorship, including
      the original version of the Work and any modifications or additions
      to that Work or Derivative Works thereof, that is intentionally
      submitted to Licensor for inclusion in the Work by the copyright owner
      or by an individual or Legal Entity authorized to submit on behalf of
      the copyright owner. For the purposes of this definition, "submitted"
      means any form of electronic, verbal, or written communication sent
      to the Licensor or its representatives, including but not limited to
      communication on electronic mailing lists, source code control systems,
      and issue tracking systems that are managed by, or on behalf of, the
      Licensor for the purpose of discussing and improving the Work, but
      excluding communication that is conspicuously marked or otherwise
      designated in writing by the copyright owner as "Not a Contribution."

      "Contributor" shall mean Licensor and any individual or Legal Entity
      on behalf of whom a Contribution has been received by Licensor and
      subsequently incorporated within the Work.

   2. Grant of Copyright License. Subject to the terms and conditions of
      this License, each Contributor hereby grants to You a perpetual,
      worldwide, non-exclusive, no-charge, royalty-free, irrevocable
      copyright license to reproduce, prepare Derivative Works of,
      publicly display, publicly perform, sublicense, and distribute the
      Work and such Derivative Works in Source or Object form.

   3. Grant of Patent License. Subject to the terms and conditions of
      this License, each Contributor hereby grants to You a perpetual,
      worldwide, non-exclusive, no-charge, royalty-free, irrevocable
      (except as stated in this section) patent license to make, have made,
      use, offer to sell, sell, import, and otherwise transfer the Work,
      where such license applies only to those patent claims licensable
      by such Contributor that are necessarily infringed by their
      Contribution(s) alone or by combination of their Contribution(s)
      with the Work to which such Contribution(s) was submitted. If You
      institute patent litigation against any entity (including a
      cross-claim or counterclaim in a lawsuit) alleging that the Work
      or a Contribution incorporated within the Work constitutes direct
      or contributory patent infringement, then any patent licenses
      granted to You under this License for that Work shall terminate
      as of the date such litigation is filed.

   4. Redistribution. You may reproduce and distribute copies of the
      Work or Derivative Works thereof in any medium, with or without
      modifications, and in Source or Object form, provided that You
      meet the following conditions:

      (a) You must give any other recipients of the Work or
          Derivative Works a copy of this License; and

      (b) You must cause any modified files to carry prominent notices
          stating that You changed the files; and

      (c) You must retain, in the Source form of any Derivative Works
          that You distribute, all copyright, patent, trademark, and
          attribution notices from the Source form of the Work,
          excluding those notices that do not pertain to any part of
          the Derivative Works; and

      (d) If the Work includes a "NOTICE" text file as part of its
          distribution, then any Derivative Works that You distribute must
          include a readable copy of the attribution notices contained
          within such NOTICE file, excluding those notices that do not
          pertain to any part of the Derivative Works, in at least one
          of the following places: within a NOTICE text file distributed
          as part of the Derivative Works; within the Source form or
          documentation, if provided along with the Derivative Works; or,
          within a display generated by the Derivative Works, if and
          wherever such third-party notices normally appear. The contents
          of the NOTICE file are for informational purposes only and
          do not modify the License. You may add Your own attribution
          notices within Derivative Works that You distribute, alongside
          or as an addendum to the NOTICE text from the Work, provided
          that such additional attribution notices cannot be construed
          as modifying the License.

      You may add Your own copyright statement to Your modifications and
      may provide additional or different license terms and conditions
      for use, reproduction, or distribution of Your modifications, or
      for any such Derivative Works as a whole, provided Your use,
      reproduction, and distribution of the Work otherwise complies with
      the conditions stated in this License.

   5. Submission of Contributions. Unless You explicitly state otherwise,
      any Contribution intentionally submitted for inclusion in the Work
      by You to the Licensor shall be under the terms and conditions of
      this License, without any additional terms or conditions.
      Notwithstanding the above, nothing herein shall supersede or modify
      the terms of any separate license agreement you may have executed
      with Licensor regarding such Contributions.

   6. Trademarks. This License does not grant permission to use the trade
      names, trademarks, service marks, or product names of the Licensor,
      except as required for reasonable and customary use in describing the
      origin of the Work and reproducing the content of the NOTICE file.

   7. Disclaimer of Warranty. Unless required by applicable law or
      agreed to in writing, Licensor provides the Work (and each
      Contributor provides its Contributions) on an "AS IS" BASIS,
      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
      implied, including, without limitation, any warranties or conditions
      of TITLE, NON-INFRINGEMENT, MERCHANTABILITY, or FITNESS FOR A
      PARTICULAR PURPOSE. You are solely responsible for determining the
      appropriateness of using or redistributing the Work and assume any
      risks associated with Your exercise of permissions under this License.

   8. Limitation of Liability. In no event and under no legal theory,
      whether in tort (including negligence), contract, or otherwise,
      unless required by applicable law (such as deliberate and grossly
      negligent acts) or agreed to in writing, shall any Contributor be
      liable to You for damages, including any direct, indirect, special,
      incidental, or consequential damages of any character arising as a
      result of this License or out of the use or inability to use the
      Work (including but not limited to damages for loss of goodwill,
      work stoppage, computer failure or malfunction, or any and all
      other commercial damages or losses), even if such Contributor
      has been advised of the possibility of such damages.

   9. Accepting Warranty or Additional Liability. While redistributing
      the Work or Derivative Works thereof, You may choose to offer,
      and charge a fee for, acceptance of support, warranty, indemnity,
      or other liability obligations and/or rights consistent with this
      License. However, in accepting such obligations, You may act only
      on Your own behalf and on Your sole responsibility, not on behalf
      of any other Contributor, and only if You agree to indemnify,
      defend, and hold each Contributor harmless for any liability
      incurred by, or claims asserted against, such Contributor by reason
      of your accepting any such warranty or additional liability.

   END OF TERMS AND CONDITIONS

   APPENDIX: How to apply the Apache License to your work.

      To apply the Apache License to your work, attach the following
      boilerplate notice, with the fields enclosed by brackets "[]"
      replaced with your own identifying information. (Don't include
      the brackets!)  The text should be enclosed in the appropriate
      comment syntax for the file format. We also recommend that a
      file or class name and description of purpose be included on the
      same "printed page" as the copyright notice for easier
      identification within third-party archives.

   Copyright 2023 Intel Corporation. All Rights Reserved.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
  
</details>
