# Intel® RealSense™ SDK for Linux ROS Samples

## Features
These samples illustrate how to develop OSRF&reg; ROS* applications using Intel® RealSense™ cameras for Object Library (OR), Person Library (PT), and Simultaneous Localization And Mapping (SLAM).

## Installation Instructions

The Intel RealSense SDK for Linux is used as the base for these ROS node.  Public installation information for RealSense for Ubuntu 16.04 is available at https://software.intel.com/sites/products/realsense/intro/

(todo: add public install instructions)

## ROS Node Samples
- [Camera](realsense_ros_camera/README.md): This ROS node (fill in content).
- [Tracking](realsense_ros_object/README.md): This ROS node (fill in content).
- [Person](realsense_ros_person/README.md): This ROS node (fill in content).
- [SLAM](realsense_ros_slam/README.md): This ROS node (fill in content).

### Run SLAM (old, move to realsense_ros_slam/README.md)

To run the slam engine:
```bash
$ cd catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch rs_slam_test camera_slam_nodelet.launch
```

To see estimated pos, in another window:
```bash
$ cd catkin-ws
$ source devel/setup.bash
$ rostopic echo /pose2d
```

## License
Copyright 2017 Intel Corporation

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this project except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

**Other names and brands may be claimed as the property of others*
