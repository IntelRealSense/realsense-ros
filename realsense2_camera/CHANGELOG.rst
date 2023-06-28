^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense2_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.54.1 (2023-06-27)
-------------------
* Applying AlignDepth filter after Pointcloud
* Publish /aligned_depth_to_color topic only when color frame present
* Support Iron distro
* Protect empty string dereference
* Fix: /tf and /static_tf topics' inconsistencies
* Revamped the TF related code
* Fixing TF frame links b/w multi camera nodes when using custom names
* Updated TF descriptions in launch py and readme
* Fixing /tf topic has only TFs of last started sensor
* add D430i support
* Fix Swapped TFs Axes
* replace stereo module with depth module
* use rs2_to_ros to replace stereo module with depth moudle
* calculate extriniscs twice in two opposite ways to save inverting rotation matrix
* fix matrix rotation
* Merge branch 'ros2-development' into readme_fix
* invert translation
* Added 'publish_tf' param in rs launch files
* Indentation corrections
* Fix: Don't publish /tf when publish_tf is false
* use playback device for rosbags
* Avoid configuring dynamic_tf_broadcaster within tf_publish_rate param's callback
* Fix lower FPS in D405, D455
* update rs_launch.py to support enable_auto_exposure and manual exposure
* fix timestamp calculation metadata header to be aligned with metadata json timestamp
* Expose USB port in DeviceInfo service
* Use latched QoS for Extrinsic topic when intra-process is used
* add cppcheck to GHA
* Fix Apache License Header and Intel Copyrights
* apply copyrights and license on project
* Enable intra-process communication for point clouds
* Fix ros2 parameter descriptions and range values
* T265 clean up
* fix float_to_double method
* realsense2_camera/src/sensor_params.cpp
* remove T265 device from ROS Wrapper - step1
* Enable D457
* Fix hdr_merge filter initialization in ros2 launch
* if default profile is not defined, take the first available profile as default
* changed to static_cast and added descriptor name and type
* remove extra ';'
* remove unused variable format_str
* publish point cloud via unique shared pointer
* make source backward compatible to older versions of cv_bridge and rclcpp
* add hdr_merge.enable and depth_module.hdr_enabled to rs_launch.py
* fix compilation errors
* fix tabs
* if default profile is not defined, take the first available profile as default
* Fix ros2 sensor controls steps and add control default value to param description
* Publish static transforms when intra porocess communication is enabled
* Properly read camera config files in rs_launch.py
* fix deprecated API
* Add D457
* Windows bring-up
* publish actual IMU optical frame ID in IMU messages
* Publish static tf for IMU frames
* fix extrinsics calculation
* fix ordered_pc arg prefix
* publish IMU frames only if unite/sync imu method is not none
* Publish static tf for IMU frames
* add D430i support
* Contributors: Arun Prasad, Arun Prasad V, Arun-Prasad-V, Christian Rauch, Daniel Honies, Gilad Bretter, Nir Azkiel, NirAz, Pranav Dhulipala, Samer Khshiboun, SamerKhshiboun, Stephan Wirth, Xiangyu, Yadunund, nvidia

4.51.1 (2022-09-13)
-------------------
* Fix crash when activating IMU & aligned depth together
* Fix rosbag device loading by preventing set_option to HDR/Gain/Exposure
* Support ROS2 Humble
* Publish real frame rate of realsense camera node topics/publishers
* No need to start/stop sensors for align depth changes
* Fix colorizer filter which returns null reference ptr
* Fix align_depth enable/disable
* Add colorizer.enable to rs_launch.py
* Add copyright and license to all ROS2-beta source files
* Fix CUDA suffix for pointcloud and align_depth topics
* Add ROS build farm pre-release to ci

* Contributors: Eran, NirAz, SamerKhshiboun

4.0.4 (2022-03-20)
------------------
* fix required packages for building debians for ros2-beta branch

* Contributors: NirAz

4.0.3 (2022-03-16)
------------------
* Support intra-process zero-copy
* Update README
* Fix Galactic deprecated-declarations compilation warning
* Fix Eloquent compilation error

* Contributors: Eran, Nir-Az, SamerKhshiboun

4.0.2 (2022-02-24)
------------------
* version 4.4.0 changed to 4.0.0 in CHANGELOG
* add frequency monitoring to /diagnostics topic.
* fix topic_hz.py to recognize message type from topic name. (Naive)
* move diagnostic updater for stream frequencies into the RosSensor class.
* add frequency monitoring to /diagnostics topic.
* fix galactic issue with undeclaring parameters
* fix to support Rolling.
* fix dynamic_params syntax.
* fix issue with Galactic parameters set by default to static which prevents them from being undeclared.

* Contributors: Haowei Wen, doronhi, remibettan

4.0.1 (2022-02-01)
------------------
* fix reset issue when multiple devices are connected
* fix /rosout issue
* fix PID for D405 device
* fix bug: frame_id is based on camera_name
* unite_imu_method is now changeable in runtime.
* fix motion module default values.
* add missing extrinsics topics
* fix crash when camera disconnects.
* fix header timestamp for metadata messages.

* Contributors: nomumu, JamesChooWK, benlev, doronhi

4.0.0 (2021-11-17)
-------------------
* changed parameters: 
  - "stereo_module", "l500_depth_sensor" are replaced by "depth_module"
  - for video streams: <module>.profile replaces <stream>_width, <stream>_height, <stream>_fps
  - removed paramets <stream>_frame_id, <stream>_optical_frame_id. frame_ids are defined by camera_name
  - "filters" is removed. All filters (or post-processing blocks) are enabled/disabled using "<filter>.enable"
  - "align_depth" is replaced with "align_depth.enable"
  - "allow_no_texture_points", "ordered_pc" replaced by "pointcloud.allow_no_texture_points", "pointcloud.ordered_pc"
  - "pointcloud_texture_stream", "pointcloud_texture_index" are replaced by "pointcloud.stream_filter", "pointcloud.stream_index_filter"

* Allow enable/disable of sensors in runtime.
* Allow enable/disable of filters in runtime.
