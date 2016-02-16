#Intel&reg; RealSense&trade; Technology - ROS Integration 

###(ROS Indigo + Ubuntu 14.04 [64-bit])
###Installation
#####Getting the camera to work on Linux

* Clone the source from the librealsense git repository https://github.intel.com/PerCSystemsEngineering/librealsense.
* Follow the instructions at https://github.intel.com/PerCSystemsEngineering/librealsense/blob/master/doc/installation.md.
* Make sure that the software stack is installed properly and that the camera is working. This can be checked by connecting the camera to a USB3 port and running the "cpp-capture" sample program in the "librealsense/bin" folder.
If this does not work, you should first fix this issue before continuing with the ROS integration.
* Make sure /usr/local/lib is in your LD__LIBRARY_PATH.

#####Building package:

* Follow the steps in the README.md file of the <b>ros</b> repository. Setup ROS and create a local catkin workspace.
* To compile just realsense package, instead of catkin_make, execute following command
    catkin_make --pkg realsense

Successful execution of command will build target <b>“r200_camera_nodelet”</b>

Sample launch files are available in realsense/launch directory

<b>realsense_r200_launch_preset.launch</b>

<b>realsense_r200_launch_manual.launch</b>

<b>realsense_r200_launch_settings.launch</b>

### Intel&reg; RealSense&trade; R200 Nodelet
Publishing stream data from the Intel® RealSense™ R200 (DS4) camera

#### Subscribed Topics
    None

#### Published Topics

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
        Registered XYZRGB point cloud.

Infrared1 camera

    camera/infrared1/image_raw (sensor_msgs/Image)
    camera/infrared1/camera_info
        Calibration data

Infrared2 camera

    camera/infrared2/image_raw (sensor_msgs/Image)
    camera/infrared2/camera_info
        Calibration data

####Parameters

    mode (string, default: preset)
        Specify the mode to start camera streams. Mode comprises of height, width and fps. 
        Preset mode enables default values whereas Manual mode enables the specified parameter values.
    color_height (int, default: 480)
        Specify the color camera height resolution.
    color_width (int, default: 640)
        Specify the color camera width resolution.
    depth_height (int, default: 360)
        Specify the depth camera height resolution.
    depth_width (int, default: 480)
        Specify the depth camera width resolution.
    depth_fps (int, default: 60)
        Specify the color camera FPS
    depth_fps (int, default: 60)
        Specify the depth camera FPS
    enable_depth (bool, default: 1) 
        Specify if to enable or not the depth camera. 1 is true. 0 is false.
    enable_color (bool, default: 1) 
        Specify if to enable or not the color camera. 1 is true. 0 is false.
    enable_pointcloud (bool, default: 1) 
        Specify if to enable or not the point cloud camera. 1 is true. 0 is false.
    camera (string, default: "R200") 
        Specify the camera name. 
    Supported options: Here are r200 camera supported options that can be set
        COLOR_BACKLIGHT_COMPENSATION : [0, 4]
        COLOR_BRIGHTNESS : [0, 255]
        COLOR_CONTRAST : [16, 64]
        COLOR_EXPOSURE : [0, 0]
        COLOR_GAIN : [0, 256]
        COLOR_GAMMA : [100, 280]
        COLOR_HUE : [-2200, 2200]
        COLOR_SATURATION : [0, 255]
        COLOR_SHARPNESS : [0, 7]
        COLOR_WHITE_BALANCE : [2000, 8000]
        COLOR_ENABLE_AUTO_EXPOSURE : [0, 0]
        COLOR_ENABLE_AUTO_WHITE_BALANCE : [0, 1]
        R200_LR_AUTO_EXPOSURE_ENABLED : [0, 1]
        R200_LR_GAIN : [100, 1600]
        R200_LR_EXPOSURE : [0, 333]
        R200_EMITTER_ENABLED : [0, 1]
        R200_DEPTH_CONTROL_PRESET : [0, 5]
        R200_DEPTH_UNITS : [1, 2147483647]
        R200_DEPTH_CLAMP_MIN : [0, 65535]
        R200_DEPTH_CLAMP_MAX : [0, 65535]
        R200_DISPARITY_MULTIPLIER : [1, 1000]
        R200_DEPTH_CONTROL_ESTIMATE_MEDIAN_DECREMENT : [0 - 255]
        R200_DEPTH_CONTROL_ESTIMATE_MEDIAN_INCREMENT  : [0 - 255]
        R200_DEPTH_CONTROL_MEDIAN_THRESHOLD : [0 - 1023]
        R200_DEPTH_CONTROL_SCORE_MINIMUM_THRESHOLD : [0 - 1023]
        R200_DEPTH_CONTROL_SCORE_MAXIMUM_THRESHOLD : [0 - 1023]
        R200_DEPTH_CONTROL_TEXTURE_COUNT_THRESHOLD : [0 - 31]
        R200_DEPTH_CONTROL_TEXTURE_DIFFERENCE_THRESHOLD : [0 - 1023]
        R200_DEPTH_CONTROL_SECOND_PEAK_THRESHOLD : [0 - 1023]
        R200_DEPTH_CONTROL_NEIGHBOR_THRESHOLD : [0 - 1023]
        R200_DEPTH_CONTROL_LR_THRESHOLD : [0 - 2047]

####Services
    get_settings (camera/get_settings)
	To get supported camera options with current value set. It returns string in options:value format where different options are seperated by semicolon.
	

###Running the R200 nodelet

Type the following to launch the camera nodelet. You will notice the camera light up.

    $ roslaunch realsense realsense_r200_launch_preset.launch

View using RVIZ:

For color, infrared1 and infrared2 streams, you can open <b>RVIZ</b> and load the published topics.
For the point cloud steam, before loading its corresponding topic, set the camera_depth_optical_frame using following command:

    $ rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map camera_depth_optical_frame 100

You can also open RVIZ and load the provided RVIZ configuration file: realsenseRvizConfiguration.rviz.
![](docs/realsenseRvizScreenshot.png)

View using other commands:
* For color stream

    $ rosrun image_view image_view image:=/camera/color/image_raw

* For depth stream

    $ rostopic echo /camera/depth/image_raw

    $ rostopic echo /camera/depth/camera_info

* For pointcloud

    $ rosrun pcl_ros convert_pointcloud_to_image input:=/camera/depth/points output:=/my_image

    $ rosrun image_view image_view image:=/my_image

* For viewing supported camera settings with current values:

    $ rosservice call /camera/get_settings

<b>Tech and dependencies</b>
* librealsense.so

<b>System:</b>
* Linux 14.04+
* ROS Indigo
* R200 (DS4) camera

** The ROS integration has been tested on a 64bit machine with Linux 14.04 (Trusty) and ROS Indigo.

###Limitations:
Currently camera ROS node supports following formats
* Color stream:    RGB8
* Depth stream:    Y16
* Infrared stream: Y8

