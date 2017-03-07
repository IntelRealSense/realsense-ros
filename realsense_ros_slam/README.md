# ROS Node for Intel® RealSense™ SLAM Library

This package contains a ROS wrapper for Intel's SLAM library. The realsense_ros_slam package provides a solution for SLAM as a ROS nodelet. It consumes the messages sent by the realsense_ros_camera nodelet, and publishes messages for the camera pose and occupancy map.

## Hardware/Software Requirements

To use realsense_ros_slam, you need a mobile agent with a RealSense ZR300 camera.

###  Ubuntu requirements:
- Ubuntu 16.04
- gcc 4.9.3
- libeigen3-dev
- libc++ dev 

###  Required Intel libraries:
- Linux SDK
- librealsense_slam
- Link: https://securewiki.ith.intel.com/pages/viewpage.action?pageId=510951805

## Inputs and Outputs

### Subscribed Topics

`camera/fisheye/image_raw`

- Message type: `sensor_msgs::Image`
- The fisheye image with timestamp

`camera/depth/image_raw`

- Message type: `sensor_msgs::Image`
- The depth image with timestamp

`camera/fisheye/camera_info`

- Message type: `sensor_msgs::CameraInfo`
- The intrinsics of the fisheye camera

`camera/depth/camera_info` 

- Message type: `sensor_msgs::CameraInfo`
- The intrinsics of the depth camera

`camera/gyro/sample`

- Message type: `sensor_msgs::Imu`
- Gyroscope sample with timestamp

`camera/accel/sample`

- Message type: `sensor_msgs::Imu`
- Accelerometer sample with timestamp 

`camera/gyro/imu_info`

- Message type: `realsense_ros_camera::IMUInfo`
- Gyroscope intrinsics, noise and bias variances

`camera/accel/imu_info`

- Message type: `realsense_ros_camera::IMUInfo`
- Accelerometer intrinsics, noise and bias variances

`camera/extrinsics/fisheye2imu`

- Message type: `realsense_ros_camera::Extrinsics`
- Fisheye to IMU extrinsics

`camera/extrinsics/fisheye2depth`

- Message type: `realsense_ros_camera::Extrinsics`
- Fisheye to depth extrinsics
        
### Published Topics

`camera_pose`

- Message type: `geometry_msgs::PoseStamped`
- The raw camera pose, in the camera's coordinate system (right-handed, +x right, +y down, +z forward)

`reloc_pose`

- Message type: `geometry_msgs::PoseStamped`
- The relocalized camera pose, in the camera's coordinate system. Published only when a relocalization has occurred.

`pose2d`

- Message type: `geometry_msgs::Pose2D`
- The 2D camera pose, projected onto a plane corresponding to the occupancy map

`tracking_accuracy`

- Message type: `realsense_ros_slam::TrackingAccuracy`
- The current 6DoF tracking accuracy (low/medium/high/failed). Currently `high` is not used.

`map`

- Message type: `nav_msgs::OccupancyGrid`
- The occupancy map

### Parameters

`trajectoryFilename` 

- `str::string`, default: 'trajectory.ppm'
- The name of the trajectory file to output. The files will be saved in the realsense_ros_slam directory.

`relocalizationFilename` 

- `str::string`, default: 'relocalization.bin'
- The name of re-localization file to output. The files will be saved in the realsense_ros_slam directory.

`occupancyFilename`
- `str::string`, default: 'occupancy.bin'
- The name of occupancy data file to output. 

`resolution`
- `float`
- Sets the size of the grid squares in the occupancy map, in meters.

The files will be saved in the realsense_ros_slam directory.

## Usage

To run the slam engine:
```bash
$ cd catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch realsense_ros_salm demo_slam.launch
```

To run the slam engine using a recorded bag file:
```bash
$ cd catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch realsense_ros_slam demo_slam_from_bag.launch bag_path:=~/test.bag
```

To print the estimated pose messages, in another console window:
```bash
$ cd catkin-ws
$ source devel/setup.bash
$ rostopic echo camera_pose
```

The `demo_slam.launch` and `demo_slam_from_bag.launch` files will automatically start rviz using a configuration file located at `launch/demo_settings.rviz`. The raw camera pose, occupancy map, and odometry are shown in rviz. The odometry message is only sent by the SLAM nodelet for demo purposes, since the `pose2d` message cannot be displayed by rviz. The odometry message contains the same 2D pose that the `pose2d` message does. This shows where the robot is located relative to the occupancy map.


