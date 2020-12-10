^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense2_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.17 (2020-09-09)
-------------------

2.2.16 (2020-08-06)
-------------------

2.2.15 (2020-07-13)
-------------------
* Merge remote-tracking branch 'origin/development' into development
* Merge pull request `#1126 <https://github.com/intel-ros/realsense/issues/1126>`_ from mcamurri/add-d435i-xacro
  add D435i modules, urdf and launchfile
* comply to ROS Noetic xacro rules (backcompatible with ROS Melodic)
* Merge branch 'development' into add-d435i-xacro
* Merge branch 'development' of https://github.com/IntelRealSense/realsense-ros into development
  realsense2_description/urdf/test_d435_multiple_cameras.urdf.xacro
* move xml start line in the right place
* add D435i modules, urdf and launchfile
* fix use_nominal_extrinsics arg/property
* add D435i modules, urdf and launchfile
* Contributors: Marco Camurri, doronhi

Forthcoming
-----------
* Add urdf for L515.
* remove librealsense2 and realsense2_camera dependencies
* Add models for D415, D435, D435i.
  For visualization, can be presented using view_model.launch.py
* fix view_d435_model.launch.py and view_d435i_model.launch.py
  run: ros2 launch realsense2_description view_d435i_model.launch.py
* Contributors: Ryan Shim, doronhi

2.2.14 (2020-06-18)
-------------------
* fix urdf issues (arg use_nominal_extrinsics).
* Add noetic support: 
  - urdf files.
  - change state_publisher into robot_state_publisher
* correct offset between camera_link and base_link
* Contributors: Brice, Marco Camurri, doronhi

* upgrade version to 2.2.13
