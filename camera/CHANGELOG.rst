^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2016-03-17)
------------------
* Convert command line args to ROS params
  This also fixes Issue #9 for publish_tf and enable_tf
  Updated README to indicate changes in R200_DISPARITY_MULTIPLIER.
* New Feature to dynamically enable/disable depth stream
  Note: Enable/disable of depth stream also enables/disables infrared streams.
* Update camera_info msgs
  Add identity matrix for rotation
  Add depth to color translation to projection matrix
* Add rgbd_launch as run dependency.
* Add missing Change Log history file
  This is a required file as part of the ROS (bloom) release process
  which was missed from our initial release.
* Contributors: Mark D Horn, Matthew Hansen, Reagan Lopez, Rajvi Jingar

1.0.0 (2016-02-29)
------------------
* Initial Release
* Contributors: Rajvi Jingar, Reagan Lopez
