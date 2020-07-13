^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense2_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.15 (2020-07-13)
-------------------
* Check runtime version of librealsense2 vs. compiled version and issue a warning is mismatch occurs.
* Support both L515 and L515 pre-prq versions.
* set infra, fisheye, IMU and pose streams to be false by default.
* add d435i-xacro
* comply to ROS Noetic xacro rules (backcompatible with ROS Melodic) 
* Contributors: Marco Camurri, doronhi

2.2.14 (2020-06-18)
-------------------
* Fix compatibility with Librealsense2 Version 2.35.2.
* Fix support for L515.
* Fix urdf issues.
* Add noetic support: change state_publisher into robot_state_publisher
* fix distortion correction model for T265 (equidistant)
* fix stability issues. Stop sensors at program termination.
* Contributors: Brice, Helen Oleynikova, doronhi

* upgrade version to 2.2.13
* fix ctrl-C closing issues.
* handle device creation exceptions.
* support LiDAR camera L515.
* optimize pointcloud. Contributors: Davide Faconti
* fix usb port id parsing issues.
* Add eigen dependency - missing for Melodic. Contributors: Antoine Hoarau
