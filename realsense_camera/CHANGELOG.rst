^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.0 (2016-10-27)
------------------
* Set DC defaults based on the configured preset (#132)
* Added initial support for ZR300 camera
* Contributors: Mark Horn, Reagan Lopez

1.5.0 (2016-09-22)
------------------
* Remove obsolete realsense_navigation files.
* Add depth control preset option (#106)
* Modify launch files for topic remapping.
* Use node handles to enable easier remapping
* Remove invalid SR300 Camera option
* Added multiple cameras support for camera power services
* Added services to start and stop the camera (#85)
* Added a RGBD launch file for SR300
* Clean-up CMakeLists.txt for librealsense
* Contributors: Amber Elliot, Kevin C Wells, Mark D Horn, Séverin Lemaignan, Tully Foote, Rajvi Jingar

1.4.0 (2016-08-19)
------------------
* Updated Install Instructions for ROS Packages
* Added Errata for F200/SR300 Camera Types
* Make librealsense pkg required
* Added code to read depth scale from camera (intel-ros/realsense#46)
* Fix SR300 Max Z + Type casting
* Updated default values for SR300 camera options
* Updated documentation with SR300 camera details
* Updated tests to include SR300 distortion parameters
* Added fix to remove blurriness from SR300 IR stream
* Added initial support for SR300 cameras (#6)
* Change to Static Transforms for camera (#84)
* Contributors: Mark D Horn, Reagan Lopez, Salah-Eddine Missri, Lincoln Lorenz

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
