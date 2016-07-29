^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.0 (2016-07-28)
------------------
* Fix the Install for Includes
* Move header files
* Updated README for F200 cameras
* Added initial support for F200 cameras
* Refactored code for new cameras
* Contributors: Mark D Horn, Reagan Lopez, Yuki Furuta

1.2.1 (2016-07-13)
------------------
* Fix starting /camera/get_settings Service
* Correct ROS Dependencies Install command

1.2.0 (2016-06-30)
------------------
* Update for ROS librealsense Package Release
* Added navigation package changes related to camera package refactor
* Updated artifacts to disable native pointcloud by default
* Refactored R200 code into derived class
* Added polling for camera
* Refactored launch and test files
* Contributors: Mark D Horn, Matthew Hansen, Reagan Lopez, Rajvi Jingar

1.1.0 (2016-06-03)
------------------
* Fix white space issues in test files
* Updated docs with Kinetic Kame details and for consistency
* Added usb_port_id for selecting camera
* Updated artifacts to reflect 'stable' tag
* Added code to skip publishing PointCloud frame if Depth and/or Color has duplicate frames
* Added code to skip publishing duplicate frames for native streams
* Added example for launching multiple cameras from a single launch file (#22)
* Fixed transformation origin bug for base frame to depth frame
* Rename package directory (#33)
* Update README to include rosdep install
* Resolved testTransform unit test issue
* Refined the log messages and made them consistent
* Removed extra space before ROS Log function calls
* Added nodelet name to log messages
* Remove hard-coded paths
* Fixed README bug to show correct depth format Z16
* Fixed unit conversion bug in the projection matrix
* Added unit test to check camera_info distortion-parameter
* adding D to camera info
* Contributors: Mark D Horn, Matthew Hansen, Reagan Lopez, Rajvi Jingar, Natalia Lyubova, Michael Gorner

1.0.4 (2016-04-25)
------------------
* Updated RGBD Launch File for Manual Mode (#25)
* Updated Documentation with Backend and ROS RealSense details (#16)
* Modified variable names to match ROS standards
* Contributors: Mark D Horn, Reagan Lopez

1.0.3 (2016-04-11)
------------------
* Updated code to enable multi-camera functionality (#7)
* Updated README and Issue Template with BKC
* Contributors: Reagan Lopez

1.0.2 (2016-03-28)
------------------
* Added functionality to access camera using Serial No (#18)
* Modified all parameters to lowercase for consistency (#13)
* Removed support for R200_DISPARITY_MULTIPLIER camera option
* Added missing install targets (#2 #17)
* Contributors: Reagan Lopez, Rajvi Jingar

1.0.1 (2016-03-17)
------------------
* Convert command line args to ROS params (#9)
* New Feature to dynamically enable/disable depth stream
* Update camera_info msgs
* Add rgbd_launch as run dependency.
* Add missing Change Log history file
* Contributors: Mark D Horn, Matthew Hansen, Reagan Lopez, Rajvi Jingar

1.0.0 (2016-02-29)
------------------
* Initial Release
* Contributors: Rajvi Jingar, Reagan Lopez
