realsense_camera (draft)

This package contains a ROS wrapper for Intel's realsense library. The realsense_camera package provides a solution to get the depth and color stream of realsense device as a ROS nodelet. For instance,it's used only with RealSense ZR300 and R200. The objectif of this module is to provides the data and stream needed by SLAM and PT ros wrapper of Intel.

1. Hardwarei/Software Requirement
  To use realsense_camera, you need a realsense device(RealSense ZR300 or LR200).
  ubuntu requirements:
    Ubuntu 16.04
    gcc 4.9.3
  libraries of intel used(version beta3 RC5):
    librealsense
    Link: https://securewiki.ith.intel.com/pages/viewpage.action?pageId=510951805
2. Example 
   To start realsense_camera with command: 
      roslaunch realsense_camera slam_camera.launch
3. Nodelet
  3.1 realsense_camera 
    The realsense_camera nodelet get the fisheye image, imu data, color image,depth image from ZR300, or depth image, color image from LR200, then send the informations. For now, the image size is:
      -depth: 320x240 30fps
      -color: 640x480 30fps
      -fisheye: 640x480 30fps
  3.1.1 Published Topics
    camera/depth/camera_info (sensor_msgs/CameraInfo)
      The intrinsics (and extrinsics) of camera( depth and depth to color)
    camera/color/camera_info (sensor_msgs/CameraInfo)
      The intrinsics of camera(color)
    camera/depth/image_raw (sensor_msgs/Image)
      The depth stream recieved from realsense device
    camera/color/image_raw (sensor_msgs/Image)
      The color stream recieved from realsense device
  3.1.2 Parameters
    ~serial_no: (str::string, default: '') 
      The serial_number of device that you want to connect. By default, the first device will be used.
  
  3.1.3 ZR300 only
    3.1.3.1 Published Topics
      camera/fisheye/camera_info (sensor_msgs/CameraInfo)
        The intrinsics of camera (fisheye)
      camera/fisheye/image_raw (sensor_msgs/Image)
        The fisheye stream recieved from realsense device
      camera/imu/gyro (realsense_camera/MotionInfo) 
        Contain the imu data of Gyroscope, timestamp and framenumber corresponding recieved from ZR300 device
      camera/imu/accel (realsenese_camera/MotionInfo)
        Contain the imu data of Accelerometer, timestamp and framenumber corresponding recieved frome ZR300 device
      camera/fisheye/fisheye_stream_and_info (realsense_camera::StreamInfo)
        Contain the fisheye stream, timestamp and framenumber corresponding recieved from ZR300 device
      camera/depth/depth_stream_and_info (realsense_camera::StreamInfo)
        Contain the depth stream, timestamp and framenumber corresponding recieved from ZR300 device
    3.1.3.2 Services
      camera/get_imu_info (realsense_camera::GetIMUInfo)
        No input needed, give back the motion intrinsic needed by slam middleware of Intel
      camera/get_fe_extrinsics (realsense_camera::GetFExtrinsics)
        No input needed, give back the fisheye to motion extrinsic and the depth to fisheye extrinsic needed by slam middleware of Intel
