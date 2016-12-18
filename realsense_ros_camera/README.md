realsense_ros_camera
=========================================
This package provides a ROS node for using the Intel® RealSense™ R200, LR200, and ZR300 cameras.

# Features
- Color RGB stream, up to 1920x1080 resolution
- Depth 16bit stream, up to 640x480 resolution
- Fisheye 8bit monochromotic stream, up to 640x480 resolution [ZR300 only]
- IMU accel and gyro data streams [ZR300 only]

# Firmware
To ensure you camera has the most current, supported firmware, please review the librealsense compatible device information. If the camera requires a firmware upgrade, please refer to the Intel® RealSense™ Camera software support page. 
*Note*: Currently there is no native Linux tool for FW updates; all updates require a system with Microsoft Windows.

# Supported Camera Types
Intel RealSense R200
Intel RealSense LR200
Intel RealSense ZR300

# ROS API
## realsense_ros_camera 
The Intel RealSense camera driver

###  3.1.1 Published Topics

**Color camera**
- color/camera_info (sensor_msgs/CameraInfo) [Latched] Camera calibration and metadata.
- color/image_raw (sensor_msgs/Image) Color rectified image, RGB format.

**Depth camera**
- depth/camera_info (sensor_msgs/CameraInfo) [Latched] Camera calibration and metadata.
- depth/image_raw (sensor_msgs/Image) Raw image from device. Contains uint16 depths in mm.

**Fisheye camera**
*Available only for ZR300 cameras*
- fisheye/camera_info (sensor_msgs/CameraInfo) [Latched] Camera calibration and metadata.
- fisheye/image_raw (sensor_msgs/Image) Raw uint16 fisheye view image.

**IMU**
*Available only for ZR300 cameras*
- accel/imu_info (realsense_ros_camera/IMUInfo) [Latched] 
- accel/sample (sensor_msgs/Imu) IMU accelerometer data
- gyro/imu_info (realsense_ros_camera/IMUInfo) [Latched] 
- gyro/sample (sensor_msgs/Imu) IMU angular data

** Camera Extrinsics **
- extrinsics/fisheye2depth (realsense_ros_camera/Extrinsics) [Latched] Calibration extrinsics allowing projection from fisheye stream to depth data
- extrinsics/fisheye2imu (realsense_ros_camera/Extrinsics) [Latched] Calibration extrinsics allowing projection from fisheye stream to IMU data

### Parameters
**Static Parameters**

Parameter | Default | Description
--- | --- | ---
serial_no | string, default: blank | Specify the serial_no to uniquely connect to a camera, especially if multiple cameras are detected by the nodelet. You may get the serial_no from the info stream by launching the camear.launch file.
enable_depth | bool, default: true | Specify if to enable or not the depth camera.
depth_width | int, default: 480 | Specify the depth camera width resolution.
depth_height | int, default: 360 | Specify the depth camera height resolution.
depth_fps | int, default: 30 | Specify the depth camera FPS.
enable_color | bool, default: true | Specify if to enable or not the color camera.
color_width | int, default: 640 | Specify the color camera width resolution.
color_height | int, default: 480 | Specify the color camera height resolution.
color_fps | int, default: 30 | Specify the color camera FPS.
enable_fisheye | bool, default: true | Available only for ZR300 cameras. Specify if to enable or not the fisheye camera.
fisheye_width | int, default: 640 | Available only for ZR300 cameras. Specify the fisheye camera width resolution.
fisheye_height | int, default: 480 | Available only for ZR300 cameras. Specify the fisheye camera height resolution.
fisheye_fps | int, default: 30 | Available only for ZR300 cameras. Specify the fisheye camera FPS.

## Running the Nodlet
```bash
$ roslaunch realsense_ros_camera camera.launch
```
Sample launch files are available in the launch directory.  

## Differences between this node and realsense_camera

A full featured realsense camera node is available as open source at http://github.com/intel-ros/realsense.  This camera node is intended to be a smaller and more compact node illustrating use of the included middleware.  
- This node is provided as sample source code only, and is not available pre-built as a ROS package.
- This node only supports the R200, LR200, and ZR300 cameras
- This node does not support use with a RGBD launch script
- This node does not support dynamic configuration of camera parameters
- This node does not export use of IR streams from camera

