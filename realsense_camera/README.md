#Intel&reg; RealSense&trade; Technology - ROS Integration
## Supported Camera Types: R200, F200, SR300 and ZR300
###Configuration:
| Version        | Best Known (indigo)  | Best Known (kinetic) |
|:-------------- |:---------------------|:---------------------|
| OS             | Ubuntu 14.04.4 LTS   | Ubuntu 16.04 LTS     |
| Kernel         | 4.4.0-34-generic     | 4.4.0-34-generic     |
| Backend        | video4linux          | video4linux          |
| librealsense   | 1.11.0               | 1.11.0               |
| R200 Firmware  | 1.0.72.06            | 1.0.72.06            |
| F200 Firmware  | 2.60.0.0             | 2.60.0.0             |
| SR300 Firmware | 3.10.10.0            | 3.10.10.0            |
| ZR300 Firmware | Camera: 2.0.71.20, Adapter: 1.27.0.0, Motion Module: 1.23.0.0 | Camera: 2.0.71.20, Adapter: 1.27.0.0, Motion Module: 1.23.0.0 |

###Installation:
To begin with, ensure that ROS is installed. (See [wiki.ros.org/ROS/Installation](http://wiki.ros.org/ROS/Installation) for instructions).
Then, either install the realsense_camera ROS debian package or build it from source.

####Installing the ROS Debian Package
* Install the realsense_sense camera package. Replace `<*distro*>` with `kinetic` or `indigo`.

  `sudo apt-get install ros-<*distro*>-realsense-camera`

  This will also install the required `ros-<*distro*>-librealsense` library on your system.

* librealsense 0.9.2 and earlier require a few extra, manual steps to enable the use of the library.
  * Clone the librealsense GitHub repository:
  ```
  git clone https://github.com/IntelRealSense/librealsense.git
  cd librealsense
  git checkout -b b0.9.2 v0.9.2
  ```
  * Follow the instructions for [Video4Linux
  backend](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md#video4linux-backend) **ONLY**.
  Only one of the three options for Step 3 should be completed

####Building the Package from Source
**NOTE:** Ensure that the compatible librealsense version specifed in the Configuration section is installed.
* Create a local catkin workspace (see [wiki.ros.org](http://wiki.ros.org/) for instructions).
* Clone the source from https://github.com/intel-ros/realsense.git.
* Run `rosdep install --skip-keys=librealsense --from-paths -i <src-dir of cloned realsense repo>` to install package dependencies.
* Compile the realsense_camera package by executing the `catkin_make` command.
  Successful execution of command will build target <b>“realsense_camera_nodelet”</b>

Sample launch files are available in [launch](launch/)

###Package Features:

####Subscribed Topics
    None

####Published Topics (default)
Color camera

    camera/color/image_raw (sensor_msgs/Image)
        Color rectified image. RGB format.
    camera/color/camera_info
        Calibration data

Depth camera

    camera/depth/image_raw (sensor_msgs/Image)
        uint16 depths in mm
    camera/depth/camera_info
        Calibration data
    camera/depth/points (sensor_msgs/PointCloud2)
        Registered XYZRGB point cloud. By default, pointcloud is disabled.

IR camera

    camera/ir/image_raw (sensor_msgs/Image)
    camera/ir/camera_info
        Calibration data

IR2 camera: Available only for <b>R200</b> and <b>ZR300</b> cameras.

    camera/ir2/image_raw (sensor_msgs/Image)
    camera/ir2/camera_info
        Calibration data

Fisheye camera: Available only for <b>ZR300</b> cameras.

    camera/fisheye/image_raw (sensor_msgs/Image)
    camera/fisheye/camera_info
        Calibration data

IMU: Available only for <b>ZR300</b> cameras.

    camera/imu/data_raw (sensor_msgs/Imu)

  Each imu topic message will either contain angular velocity or linear acceleration data.
  The element 0 of the associated covariance matrix will be set to -1 to indicate the absence of data.
  E.g. If the topic message does not contain angular velocity data, then the element 0 of the angular velocity covariance matrix will be set to -1.

  This topic message does not contain orientation data and hence the element 0 of the orientation covariance matrix will always be -1.

####Services
    get_settings (camera/driver/get_settings)
        Gets the current value of the supported camera options in "options:value" format separated by semicolon.
    set_power < true/false > (camera/driver/set_power < true/false >) # true - to power on the camera, false - to power off the camera
        Sets camera power to turn camera on/off. It turns off the camera only if camera has no subscribers. Otherwise it returns false.
    force_power < true/false > (camera/driver/force_power < true/false >) # true - to power on the camera, false - to forcefully power off the camera
        Sets camera power to turn camera on/off. It turns off the camera regardless of number of subscribers.
    is_powered (camera/driver/is_powered)
        Gets current state of camera power. It checks whether camera is on or off and returns true or false respectively.
    get_imu_info (camera/driver/get_imu_info): Available only for ZR300 cameras.
        Gets the IMU intrinsic data. Contains the custom IMUInfo.msg for the "accel" and "gyro" IMU sensors.
   Refer to [IMUInfo.msg Definition](msg/IMUInfo.msg)

####Transform Frames
    rostopic echo /tf_static
        View the static camera transforms generated by the camera nodelet.
  You may refer to the [ROS tf wiki](http://wiki.ros.org/tf) for more commands.

####Static Parameters

        serial_no (string, default: blank)
            Specify the serial_no to uniquely connect to a camera, especially if multiple cameras are detected by the nodelet.
            You may get the serial_no from the info stream by launching the default launch file.
        usb_port_id (string, default: blank)
            Alternatively to serial_no, this can be used to connect to a camera by its USB Port ID, which is a 
            Bus Number-Port Number in the format "Bus#-Port#". If used with serial_no, both must match correctly for 
            camera to be connected.
        camera_type (string, default: blank)
            Specify the type of the camera - "R200", "F200", "SR300" or "ZR300".
        mode (string, default: preset)
            Specify the mode to start camera streams. Mode comprises of height, width and fps. 
            Preset mode enables default values whereas Manual mode enables the specified parameter values.
        enable_depth (bool, default: true)
          Specify if to enable or not the depth and infrared camera(s).
          Note: Infrared stream(s) will be enabled or disabled along with depth stream.
        depth_width (int, default: 480 for R200 and ZR300, 640 for F200 and SR300)
            Specify the depth camera width resolution.
        depth_height (int, default: 360 for R200, 480 for F200 and SR300)
            Specify the depth camera height resolution.
        depth_fps (int, default: 60)
            Specify the depth camera FPS
        enable_color (bool, default: true)
            Specify if to enable or not the color camera.
        color_width (int, default: 640)
            Specify the color camera width resolution.
        color_height (int, default: 480)
            Specify the color camera height resolution.
        color_fps (int, default: 60)
            Specify the color camera FPS
        enable_fisheye (bool, default: true): Available only for ZR300 cameras.
            Specify if to enable or not the fisheye camera.
        fisheye_width (int, default: 640): Available only for ZR300 cameras.
            Specify the fisheye camera width resolution.
        fisheye_height (int, default: 480): Available only for ZR300 cameras.
            Specify the fisheye camera height resolution.
        fisheye_fps (int, default: 60): Available only for ZR300 cameras.
            Specify the fisheye camera FPS
        enable_imu (bool, default: true): Available only for ZR300 cameras.
            Specify if to enable or not the IMU sensor.
        enable_pointcloud (bool, default: false) 
            Specify if to enable or not the native pointcloud.
            By default, it is set to false due to performance issues.
        enable_tf (bool, default: true) 
            Specify if to enable or not the transform frames.
        base_frame_id (string, default: camera_link)
            Specify the base frame id of the camera.
        depth_frame_id (string, default: camera_depth_frame)
            Specify the depth frame id of the camera.
        depth_optical_frame_id (string, default: camera_depth_optical_frame)
            Specify the depth optical frame id of the camera.
        color_frame_id (string, default: camera_rgb_frame)
            Specify the color frame id of the camera.
        color_optical_frame_id (string, default: camera_rgb_optical_frame)
            Specify the color optical frame id of the camera.
        ir_frame_id (string, default: camera_ir_frame)
            Specify the IR frame id of the camera.
        ir_optical_frame_id (string, default: camera_ir_optical_frame)
            Specify the IR optical frame id of the camera.
        ir2_frame_id (string, default: camera_ir2_frame): Available only for R200 and ZR300 cameras.
            Specify the IR2 frame id of the camera.
        ir2_optical_frame_id (string, default: camera_ir2_optical_frame): Available only for R200 and ZR300 cameras.
            Specify the IR2 optical frame id of the camera.
        fisheye_frame_id (string, default: camera_fisheye_frame): Available only for ZR300 cameras.
            Specify the fisheye frame id of the camera.
        fisheye_optical_frame_id (string, default: camera_fisheye_optical_frame): Available only for ZR300 cameras.
            Specify the fisheye optical frame id of the camera.
        imu_frame_id (string, default: camera_imu_frame): Available only for ZR300 cameras.
            Specify the IMU frame id of the camera.
        imu_optical_frame_id (string, default: camera_imu_optical_frame): Available only for ZR300 cameras.
            Specify the IMU optical frame id of the camera.

####Dynamic Parameters

For <b>R200</b> cameras, refer to [r200_params.cfg](cfg/r200_params.cfg)

For <b>F200</b> cameras, refer to [f200_params.cfg](cfg/f200_params.cfg)

For <b>SR300</b> cameras, refer to [sr300_params.cfg](cfg/sr300_params.cfg)

For <b>ZR300</b> cameras, refer to [zr300_params.cfg](cfg/zr300_params.cfg)

Note: For Autoexposure edge parameters, max value will go only upto the bounds of the infrared image.
E.g. For 320x240 infrared image, valid values are within 0-319 and 0-239)

Use rqt_reconfigure GUI to view and edit the parameters that are accessible via dynamic_reconfigure.
Command to launch GUI:

    $ rosrun rqt_reconfigure rqt_reconfigure

Command to change dynamic parameters using commandline:

    $ rosrun dynamic_reconfigure dynparam set /<node> <parameter_name> <value>
    E.g. $ rosrun dynamic_reconfigure dynparam set /R200Nodelet color_backlight_compensation 2


###Running the nodelet:
####Getting Started
Use the following command to launch the camera nodelet. You will notice the camera light up.

For <b>R200</b> cameras

    $ roslaunch realsense_camera r200_nodelet_default.launch

For <b>F200</b> cameras

    $ roslaunch realsense_camera f200_nodelet_default.launch

For <b>SR300</b> cameras

    $ roslaunch realsense_camera sr300_nodelet_default.launch

For <b>ZR300</b> cameras

    $ roslaunch realsense_camera zr300_nodelet_default.launch

Sample launch files are available in [launch](launch/). 
Make sure the correct "camera_type" is specified to launch the desired camera.

View using RVIZ:

For color and infrared stream(s), you can open <b>RVIZ</b> and load the published topics.

You can also open RVIZ and load the provided RVIZ configuration file: realsenseRvizConfiguration1.rviz.
```
   $ roscd realsense_camera
   $ rosrun rviz rviz -d rviz/realsenseRvizConfiguration1.rviz
```

View using other commands:
For color stream

    $ rosrun image_view image_view image:=/camera/color/image_raw

For depth stream

    $ rostopic echo /camera/depth/image_raw

    $ rostopic echo /camera/depth/camera_info

For pointcloud

    $ rosrun pcl_ros convert_pointcloud_to_image input:=/camera/depth/points output:=/my_image

    $ rosrun image_view image_view image:=/my_image

For viewing supported camera settings with current values:

    $ rosservice call /camera/get_settings

####Launching Multiple Cameras
<b>Note:</b> The following example is based on <b>R200</b> cameras. 
Similar options are applicable to F200, SR300 and ZR300 cameras too, just by updating the "camera_type".

For running multiple cameras simultaneously:  
<b>Option 1:</b> Using single nodelet manager for all the cameras
* Use "r200_nodelet_multiple_cameras.launch".

<b>Option 2:</b> Using separate nodelet manager for each camera
* Create ".launch" files similar to "r200_nodelet_rgbd.launch" for each camera.
    * You may choose to include (or not) the "processing.launch.xml" based on your requirement.
* Launch the ".launch" files for each camera in separate terminals.

<b> For either option you must:</b>
* Update the "camera" and either the "serial_no" or "usb_port_id" argument with unique values for each camera.
  * "camera" should be a user friendly string that follows the ROS Names convention. (E.g. "camera1")
  * "serial_no" is the camera serial number and can be found by running the nodelet and viewing the terminal output
  * "usb_port_id" is Bus Number-Port Number in "Bus#-Port#" format, and can be found by using `lsusb -t` 
  * if both "serial_no" and "usb_port_id" are set, both much match the same camera

###Unit Tests:
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

###Developer API:
Refer to the following:
* [base_nodelet.h](include/realsense_camera/base_nodelet.h)
* [r200_nodelet.h](include/realsense_camera/r200_nodelet.h)
* [f200_nodelet.h](include/realsense_camera/f200_nodelet.h)
* [sr300_nodelet.h](include/realsense_camera/sr300_nodelet.h)
* [zr300_nodelet.h](include/realsense_camera/zr300_nodelet.h)

###Limitations:

* Currently, the camera nodelet only supports the following formats:
    * Color stream:    "RGB8"
    * Depth stream:    "Z16"
    * Infrared stream(s): "Y8" for R200, F200 and ZR300. "Y16" for SR300.
    * Fisheye stream: "RAW8"

* Generating a Depth Registered Point Cloud is very memory intensive.
E.g. The topic /camera/depth_registered/points, generated by launch file
"r200_nodelet_rgbd.launch", works best at 30 fps using 640x480 resolution
on a system with 16GB of RAM.

* The camera does not provide hardware based depth registration/projector data.
Hence the launch file "r200_nodelet_rgbd.launch" will not generate data for the following topics:
    * /camera/depth_registered/hw_registered/image_rect_raw
    * /camera/depth_registered/hw_registered/image_rect
    * /camera/depth_registered/image
    * /camera/depth/disparity
    * /camera/depth_registered/disparity

* The performance benchmark for multiple cameras launched at the same time has not been defined yet.

* For ZR300 cameras, the transform between base frame and imu frame has been hardcoded until librealsense
supports fetching this data during runtime.

###Errata:

* The usb-port-id logic does not work for F200 and SR300 cameras due to a known
[librealsense Issue #220](https://github.com/IntelRealSense/librealsense/issues/220).
Until it gets fixed, use serial_no when multiple F200 or SR300 cameras are connected to the system.

* Multiple cameras can only be started from a single launch file for F200 and SR300 camera types;
see [Issue #93](https://github.com/intel-ros/realsense/issues/92). This appears to be linked to the
usb-port-id [librealsense Issue #220](https://github.com/IntelRealSense/librealsense/issues/220).
Until it gets fixed, use a single launch file for multiple F200 or SR300 cameras.

* For F200 and SR300 cameras, native pointcloud is not generated even after enabling pointcloud
[Issue #89](https://github.com/intel-ros/realsense/issues/89).
Unlikely to be fixed as current plan is to
[remove native point cloud generation](https://github.com/intel-ros/realsense/issues/47).
