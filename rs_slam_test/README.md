rs_slam_test (draft)
==================================
This package contains a ROS wrapper for Intel's SLAM library. The rs_slam_test package provides a solution of SLAM as a ROS nodelet. It take the messages sended by realsense_camera nodelet. The objectif of this module is to find out map informations and 2Dpose of agent.

1. Hardwarei/Software Requirement
----------------------------------------------
  To use rs_slam_test,you need a mobile agent with a RealSense ZR200 mounted.
  ubuntu requirements:
    Ubuntu 16.04
    gcc 4.9.3
    libeigen3-dev
    libc++ dev
  libraries of intel used(version beta3 RC5):
    Linux SDK
    librealsense_slam
    Link: https://securewiki.ith.intel.com/pages/viewpage.action?pageId=510951805
2. Example 
--------------------------------------------
   To start rs_slam_test and realsense_camera with command: 
      roslaunch rs_slam_test camera_slam_nodelet.launch
3. Nodelet
---------------------------------------------
# 3.1 rs_slam_test 
    The rs_slam_test nodelet takes fisheye image, imu data, color image,depth image from device, then send the informations of "map" and "2Dpose" of agent as type of nav_msgs/OccupancyGrid and geometry_msgs/Pose2D messages after calculation.
##  3.1.1 Subscribed Topics
    camera/fisheye/camera_info (sensor_msgs/CameraInfo)
      The intrinsics of camera (fisheye)
    camera/depth/camera_info (sensor_msgs/CameraInfo)
      The intrinsics (and extrinsics) of camera( depth and depth to color)
    camera/imu/gyro (realsense_camera/MotionInfo)
      Contain the imu data of Gyroscope, timestamp and framenumber corresponding
    camera/imu/accel (realsenese_camera/MotionInfo)
      Contain the imu data of Accelerometer, timestamp and framenumber corresponding
    camera/fisheye/fisheye_stream_and_info (realsense_camera::StreamInfo)
      Contain the fisheye stream, timestamp and framenumber corresponding
    camera/depth/depth_stream_and_info (realsense_camera::StreamInfo)
      Contain the depth stream, timestamp and framenumber corresponding
##  3.1.2 Published Topics
    poseMatrix (rs_slam_test/PoseMatrix)
        6 dof matrix 
    pose2d (geometry_msgs/Pose2D)
        pose of agent in 2 dimentions(x,y,theta)
    map (nav_msgs/OccupancyGrid)
        map info of area passed
##  3.1.3 Parameters
<br /\>    ~trajectoryFilename: (str::string, default: 'trajectory.ppm') 
        name of trajectory file of agent. The files will be saved in the rs_slam_test directory
<br /\>    ~relocalizationFilename: (str::string, default: 'relocalization.bin')
        name of relocalization data file. The files will be saved in the rs_slam_test directory
<br /\>    ~occupancyFilename: (str::string, default: 'occupancy.bin')
        name of occupancy data file. The files will be saved in the rs_slam_test directory

