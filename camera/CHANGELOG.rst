^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2016-04-11)
------------------
* Updated code to enable multi-camera functionality
  Added code to generate frame_id's based on user entered values.
  Updated rgbd_launch files to reflect the changes.
  Updated README to reflect the changes.
  This fixes Issue #7.
* Updated README and Issue Template with BKC
* Contributors: Reagan Lopez

1.0.2 (2016-03-28)
------------------
* Added functionality to access camera using Serial No:
  This fixes Issue #18.
  Added code to handle various use cases for input serial number such as
  --Exit with error if no serial number is specified and multiple cameras are detected.
  --Exit with error if no camera is detected that matches the input serial number.
  --Prints all the detected cameras.
  Updated launch files with a placeholder for serial number.
  Updated rgbd_launch test code to remove hardcoded topic names.
* Modified all parameters to lowercase for consistency
  This fixes Issue #13.
* Removed support for R200_DISPARITY_MULTIPLIER camera option
* Added missing install targets
  This fixes Pull Request #2.
  This fixes Issue #17.
* Contributors: Reagan Lopez, Rajvi Jingar

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
