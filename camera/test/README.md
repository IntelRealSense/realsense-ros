#Intel&reg; ROS RealSense Package - Unit Testing
###(ROS Indigo + Ubuntu 14.04 [64-bit])
ROS node unit test (rostest + gtest): node unit tests start up "RealsenseNodelet" and test its published topics, supported parameter values and services.

#####Building package:

* Follow the steps in the README.md file of the <b>ros</b> repository. Setup ROS and create a local catkin workspace.
* To compile just realsense package, instead of catkin_make, execute following command
    catkin_make --pkg realsense

Successful execution of command will build target <b>"utest"</b> along with target <b>“r200_camera_nodelet”</b>

#### Subscribed Topics
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

####Services called
    get_settings (camera/get_settings)
        To get supported camera options with current value set. 


###Running the utest node
The Unit Tests can be executed using either of the methods:

1. Using rostest command with test files

    $ rostest <path of test file>
    e.g rostest realsense_tests_r200_depth_only.test 

2. Using rosrun command

    $ roslaunch realsense realsense_r200_launch_manual.launch

    $ rosrun realsense_tests utest <args>
    e.g rosrun realsense_tests utest enable_depth 1 depth_encoding 16UC1 depth_height 360 depth_width 480 depth_step 960 enable_color 1 color_encoding rgb8 color_height 480 color_width 640 color_step 1920

Sample testfiles are available in test directory

<b>realsense_tests_r200_color_only.test</b>

<b>realsense_tests_r200_depth_only.test</b>

<b>realsense_tests_r200_resolution.test</b>

<b>realsense_tests_r200_settings.test</b>

Both of these methods first starts "RealsenseNodelet" for Intel® RealSense™ R200 (DS4) camera and then executes all the unittests.


