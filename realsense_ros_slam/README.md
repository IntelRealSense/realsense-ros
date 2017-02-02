realsense_ros_slam released by Perc-China(draft)
==============================================
This package contains a ROS wrapper for Intel's SLAM library. The realsense_ros_slam package provides a solution of SLAM as a ROS nodelet. It take the messages sent by realsense_ros_camera nodelet. The objective of this module is to find out map information and 2Dpose of agent.

1. Hardwarei/Software Requirement
----------------------------------------------
  To use realsense_ros_slam, you need a mobile agent with a RealSense ZR200 mounted.
###  ubuntu requirements:
<br /\>     Ubuntu 16.04
<br /\>     gcc 4.9.3
<br /\>     libeigen3-dev
<br /\>     libc++ dev 
###  libraries of intel used(version beta3 RC5):
<br /\>     Linux SDK
<br /\>     librealsense_slam
<br /\>     Link: https://securewiki.ith.intel.com/pages/viewpage.action?pageId=510951805
2. Example 
--------------------------------------------
<br /\>    To start realsense_ros_slam and realsense_ros_camera with command: 
<br /\>       roslaunch realsense_ros_slam camera_slam_nodelet.launch
3. Nodelet
--------------------------------------------
## 3.1 realsense_ros_slam 
    The realsense_ros_slam nodelet takes fisheye image, imu data, color image, depth image from device, then send the information of "map" and "2Dpose" of agent as type of nav_msgs/OccupancyGrid and geometry_msgs/Pose2D messages after calculation.
###  3.1.1 Subscribed Topics
      camera/fisheye/camera_info (sensor_msgs/CameraInfo)
        The intrinsic of camera (fisheye)
      camera/depth/camera_info (sensor_msgs/CameraInfo)
        The intrinsic (and extrinsic) of camera( depth and depth to color)
      camera/imu/gyro (realsense_ros_camera/MotionInfo)
        Contain the imu data of Gyroscope, timestamp and frame number corresponding
      camera/imu/accel (realsenese_camera/MotionInfo)
        Contain the imu data of Accelerometer, timestamp and frame number corresponding
      camera/fisheye/fisheye_stream_and_info (realsense_ros_camera::StreamInfo)
        Contain the fisheye stream, timestamp and frame number corresponding
      camera/depth/depth_stream_and_info (realsense_ros_camera::StreamInfo)
        Contain the depth stream, timestamp and frame number corresponding
###  3.1.2 Published Topics
      poseMatrix (realsense_ros_slam/PoseMatrix)
          6 dof matrix 
      pose2d (geometry_msgs/Pose2D)
          pose of agent in 2 dimentions(x,y,theta)
      map (nav_msgs/OccupancyGrid)
          map info of area passed
###  3.1.3 Parameters
      ~trajectoryFilename: (str::string, default: 'trajectory.ppm') 
          name of trajectory file of agent. The files will be saved in the realsense_ros_slam directory
      ~relocalizationFilename: (str::string, default: 'relocalization.bin')
          name of re-localization data file. The files will be saved in the realsense_ros_slam directory
      ~occupancyFilename: (str::string, default: 'occupancy.bin')
          name of occupancy data file. The files will be saved in the realsense_ros_slam directory

