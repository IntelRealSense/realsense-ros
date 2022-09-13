^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense2_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
