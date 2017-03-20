realsense_ros_camera
=========================================
This package provides a ROS node for using the Intel® RealSense™ R200, LR200, and ZR300 cameras.

# Features
- Color RGB stream, up to 1920x1080 resolution
- Depth 16bit stream, up to 640x480 resolution
- Fisheye 8bit monochromotic stream, up to 640x480 resolution (ZR300 only)
- IMU accel and gyro data streams (ZR300 only)

# Firmware
To ensure you camera has the most current, supported firmware, please review the librealsense compatible device information. If the camera requires a firmware upgrade, please refer to the Intel® RealSense™ Camera software support page. 
*Note*: Currently there is no native Linux tool for FW updates; all updates require a system with Microsoft Windows.

# Supported Camera Types
Intel RealSense R200
Intel RealSense LR200
Intel RealSense ZR300

# ROS API

## realsense_ros_camera 
The Intel RealSense camera driver nodelet

### Published Topics

	/camera/color/camera_info (sensor_msgs/CameraInfo)
		Color camera calibration and metadata.	
	/camera/color/image_raw (sensor_msgs/Image) 
		Color rectified image, RGB format.
	/camera/depth/camera_info (sensor_msgs/CameraInfo)
		Depth camera calibration and metadata.
	/camera/depth/image_raw (sensor_msgs/Image) 
		Depth image, 16-bit format containing uint16 depths in mm.
	/camera/fisheye/camera_info (sensor_msgs/CameraInfo)
		(ZR300 only) Fisheye camera calibration and metadata.
	/camera/fisheye/image_raw (sensor_msgs/Image) 
		(ZR300 only) Fisheye image, 8-bit grayscale.
	/camera/accel/imu_info (realsense_ros_camera/IMUInfo) [Latched] 
		(ZR300 only) IMU accelerometer calibration and metadata.
	/camera/accel/sample (sensor_msgs/Imu) 
		(ZR300 only) IMU accelerometer data
	/camera/gyro/imu_info (realsense_ros_camera/IMUInfo) [Latched] 
		(ZR300 only) IMU angular calibration and metadata.
	/camera/gyro/sample (sensor_msgs/Imu) 
		(ZR300 only) IMU angular data
	/camera/extrinsics/fisheye2depth (realsense_ros_camera/Extrinsics) [Latched] 
		(ZR300 only) Calibration extrinsics allowing projection from fisheye stream to depth data
	/camera/extrinsics/fisheye2imu (realsense_ros_camera/Extrinsics) [Latched]
		(ZR300 only) Calibration extrinsics allowing projection from fisheye stream to IMU data

### Parameters

	~serial_no (string, default: blank)
		Specify the serial_no to uniquely connect to a camera, especially if multiple cameras are detected by the nodelet. You may get the serial_no from the info stream by launching the camear.launch file.
	~enable_depth (bool, default: true) 
		Specify if to enable or not the depth camera.
	~depth_width (int, default: 480)
		Specify the depth camera width resolution.
	~depth_height (int, default: 360)
		Specify the depth camera height resolution.
	~depth_fps (int, default: 30)
		Specify the depth camera FPS.
	~enable_color (bool, default: true)
		Specify if to enable or not the color camera.
	~color_width (int, default: 640)
		Specify the color camera width resolution.
	~color_height (int, default: 480)
		Specify the color camera height resolution.
	~color_fps (int, default: 30)
		Specify the color camera FPS.
	~enable_fisheye (bool, default: true)
		(ZR300 only) Specify if to enable or not the fisheye camera.
	~fisheye_width (int, default: 640)
		(ZR300 only) Specify the fisheye camera width resolution.
	~fisheye_height (int, default: 480)
		(ZR300 only) Specify the fisheye camera height resolution.
	~fisheye_fps (int, default: 30)
		(ZR300 only) Specify the fisheye camera FPS.

## Usage
To start the camera:
```bash
$ roslaunch realsense_ros_camera camera.launch
```

To start the camera and camera viewers to see the video stream:
 ```bash
# For R200/LR200 camera:
$ roslaunch realsense_ros_camera demo_r200_camera.launch
# For ZRZ300 camera:
$ roslaunch realsense_ros_camera demo_zr300_camera.launch
```

## Differences between this node and realsense_camera

A full featured realsense camera node is available as open source at http://github.com/intel-ros/realsense.  This camera node is intended to be a smaller and more compact node illustrating use of the included middleware.  
- This node is provided as sample source code only, and is not available pre-built as a ROS package.
- This node only supports the R200, LR200, and ZR300 cameras
- This node does not support use with a RGBD launch script
- This node does not support dynamic configuration of camera parameters
- This node does not export use of IR streams from camera

