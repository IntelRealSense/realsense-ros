^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense2_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.3 (2021-11-11)
------------------
* Add D455 description
* Add missing aluminum material to d415 model.
* Contributors: Gilad Bretter, doronhi

3.2.2 (2021-07-01)
------------------
* Added imu frames to _l515.urdf.xacro
* Add conditional param use_mesh
* Add demo_pointcloud_launch.py
* Contributors: Simon Honigmann, doronhi

3.2.1 (2021-05-10)
------------------

3.2.0 (2021-05-05)
------------------

3.1.6 (2021-05-05)
------------------

3.1.5 (2021-03-24)
------------------

3.1.4 (2021-02-18)
------------------
* d415 add plug
* fix d415 mass in _d415.urdf.xacro.
  import tests for xacro files.
* Contributors: Manuel Stahl, Tim Übelhör, doronhi

3.1.3 (2020-12-28)
------------------
* fix realsense2_description's dependency to realsense2_camera_msgs
  remove boost dependency.
  rename node_namespace to namespace
  rename node_executable to executable
* Contributors: doronhi

3.1.2 (2020-12-10)
------------------
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
