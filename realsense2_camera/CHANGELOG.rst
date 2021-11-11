^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense2_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.3 (2021-11-11)
------------------
* add parameter reconnect_timeout
* default frame_id includes namespace.
* Added dummy transformation for multi camera example
* add parameter wait_for_device_timeout
* Fix deprecation warnings when building on Rolling
* Make pointcloud_qos a configurable parameter
* show warning when requested profile cannot be selected.
* send only 4 distortion coeffs when using equidistant
* fixed missing std namespace
* Add ros2 github actions.
* add temperature diagnostics
* Add a parameter, diagnostics_period, to control if and how often will messages be published on the diagnostic topic.
* publish diagnostics topic for Asic and Projector temperature
* Support spaces in the filters Parameter string.
* publish metadata
* Add echo_metadada.py - An example script for subscribing and parsing metadata topics.
* Add service: device_info
* fixed device_name value to snake case
* Add device name.
* Contributors: Guillaume Doisy, Marenix, Nathan Brooks, Jacco van der Spek, doronhi

3.2.2 (2021-07-01)
------------------
* Support Galactic and Rolling
* Fix reading yaml config file
* No tf broadcaster object if publish_tf is set to false
* Add udev-rules installation to debian
* update ros2 launch examples (demo_t265_launch.py, rs_t265_launch.py, demo_pointcloud_launch.py)
* fix rs_multi_camera_launch.py to include default separate node names.
* Add support for L535
* QoS parameters to be applied for all publishers
* Imu_default QoS changed to hid_default
* Pointcloud_default and info_default QoS changed to qos_default
* Contributors: Guillaume Doisy, TSC21, anaelle, doronhi

3.2.1 (2021-05-10)
------------------
* Add build dependency **ros_environment**
* Contributors: doronhi

3.2.0 (2021-05-05)
------------------
* Added filling correct Tx, Ty values in projection matrix of right camera.
* Fixed frame_id of right sensor to match left sensor in a stereo pair.pair
* Contributors: Pavlo Kolomiiets, doronhi

3.1.6 (2021-05-05)
------------------
* Fix pointcloud message size when no texture is added.
* migration toggle_sensors feature to ros2
* Add option for aligning pointcloud to color image.
* removed option to align to other streams other then color.
* Contributors: anaelle, doronhi

3.1.5 (2021-03-24)
------------------
* Support Eloquent and Dashing.
* Add filter: HDR_merge
* fix initialization of colorizer image if user specified negative image size (as is in default launch file)
* Remove the following tests for known playback issue with librealsense2 version 2.43.0: align_depth_color_1, align_depth_ir1_1, align_depth_ir1_decimation_1.
* Remove wrong dependency
* changed default image QOS to SYSTEM_DEFAULT
* Add missing librealsense2 dependency from package.xml
* fix bug: selection of profile disregarded stream index.
* Contributors: changseung.yu, doronhi

3.1.4 (2021-02-18)
------------------
* fix reading json file with device other than D400 series.
* Publish depth confidence image for supporting devices (L515)
* Add selecting QoS option
* Import unit-tests
* fix timestamp domain issues
  - Add offset to ros_time only if device uses hardware-clock. Otherwise use device time - either system_time or global_time.
  - Warn of a hardware timestamp possible loop.
* Choose the default profile in case of an invalid request.
* Avoid aligning confidence image.
* Add an option for an Ordered PointCloud.
* Contributors: Gabriel Urbain, Isaac I.Y. Saito, Itamar Eliakim, Marc Alban, doronhi

3.1.3 (2020-12-28)
------------------
* Publish depth confidence image for supporting devices (L515)
* fix bug: dynamic tf publisher.
* fix realsense2_description's dependency to realsense2_camera_msgs
* remove boost dependency.
* rename node_namespace to namespace
* rename node_executable to executable
* Contributors: doronhi

3.1.2 (2020-12-10)
------------------
* upgrade librealsense2 version to 2.40.0
* Added pointcloud attributes, when RS2_STREAM_ANY is enabled
* colorize the aligned depth image.
  fix missing parameters description.
* add infra_rgb option: enable streaming D415's infra stream as RGB.
  Expose stereo extrinsics
  Avoid currently unsupported Z16H depth format.
  Enable launch cmd line params
  Add notification if connected using USB2.1 port.
* Fix README.md
* Add launch option: output:=[screen|log]: enable sending logs to ros log file.
* rename rs.d400_and_t265.launch.py to rs_d400_and_t265_launch.py
  add rs_multi_camera_launch.py on top of rs_launch.py
* Remove '_' prefix from parameter serial_no. Allow to pass serial_no which is sometimes a string containing only digits.
* remove redundant nav_msgs dependency (caused build error)
* remove node package, change msg package name
* update README.md
* Contributors: Ryan Shim, doronhi

3.1.1 (2020-09-23)
------------------
* fix bug: Conversion from milliseconds to nanoseconds.
  enable use of parameter: use_sim_time.
* various fixes for canonical ROS2
* Contributors: AustinDeric, doronhi

3.1.0 (2020-09-16)
------------------
* port support of T265 from ROS1.
* Contributors: doronhi

3.0.0 (2020-09-16)
------------------
* Update README.md
* Enable recovery from reconnect event.
* Add an example config file: d435i.yaml.
* fix launch file installation. Allow running the following command: 'ros2 launch realsense2_camera rs.launch.py'
* enable set_auto_exposure_roi options.
* enable sensors and filters dynamic parameters.
* clean
* RealSenseNodeFactory inherits from rclcpp::Node
* Add project: realsense2_node
* Add support for D455
* README.md: update installation process.
