^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense2_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
