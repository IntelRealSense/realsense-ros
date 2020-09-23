^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense2_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
