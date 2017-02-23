#Intel&reg; RealSense&trade; Camera Driver for ROS

Please reference the documentation on the [ROS realsense_camera wiki](http://wiki.ros.org/realsense_camera).

##Unit Tests:
The Unit Tests can be executed using either of the methods:

Using `rostest` command with test files

    $ rostest realsense_camera <test_filename>
    E.g. rostest realsense_camera r200_nodelet_disable_color.test 

Using `rosrun` command

    $ roslaunch realsense_camera r200_nodelet_modify_params.launch

    $ rosrun realsense_camera realsense_camera_test <args>
    E.g. rosrun realsense_camera realsense_camera_test enable_depth 1 depth_encoding 16UC1 depth_height 360 depth_width 480 depth_step 960 enable_color 1 color_encoding rgb8 color_height 480 color_width 640 color_step 1920

Both these methods first starts the nodelet and then executes all the unit tests.

Sample test files are available in "realsense_camera/test" directory

##Bat Tests:
The bat tests can be executed using `rostest` command with test files

    $ rostest realsense_camera <test_filename>
    E.g. rostest realsense_camera r200_camera_info_matrix_check.test

Sample test files are available in "realsense_camera/test/files/bat-tests" directory.

Bat tests include its executable python scripts, they will be copied to specific directory when build realsense_camera.
If will not run the bat tests again or want to clean realsense_camera, please remove the bat python scripts by additional running `catkin_make clean-script` command.

##Errata:
See the [GitHub Issues Bugs](https://github.com/intel-ros/realsense/labels/bug)
for a complete list.

* F200/SR300 cameras:
[Multiple cameras can only be started from a single launch file for F200 and SR300 camera types.]
(https://github.com/intel-ros/realsense/issues/92)

* F200/SR300 cameras:
[Native pointcloud is not generated even after enabling pointcloud.]
(https://github.com/intel-ros/realsense/issues/89) This is unlikely to be fixed
as current plan is to [remove native point cloud generation]
(https://github.com/intel-ros/realsense/issues/47) from the node.
