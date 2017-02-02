# Intel® RealSense™ SDK for Linux ROS Samples

## Features
These samples illustrate how to develop OSRF&reg; ROS* applications using Intel® RealSense™ cameras for Object Library (OR), Person Library (PT), and Simultaneous Localization And Mapping (SLAM).

## Functionality
**API is experimental and not an official Intel product. It is subject to incompatible API changes in future updates. Breaking API changes are noted through major release numbers**

The following sample projects are provided in this release:
- Object Recognition
- Person Tracking
- SLAM

 
## Installation Instructions

The Intel RealSense SDK for Linux is used as the base for everything used by these modules.  Public installation information for RealSense for Ubuntu 16.04 is available at https://software.intel.com/sites/products/realsense/intro/

Using the instructions documented above, see 'script_setup.sh' in this repo to install everything needed from scratch, from clean Ubuntu install including ROS and the RealSense SDK for Linux:
```bash
$ wget https://github.intel.com/raw/IntelRealSense/realsense_ros/master/script_setup.sh
$ chmod a+x ./script_setup.sh
$ ./script_setup.sh
```

**Note**: These instructions above currently use a different librealsense than provided publically as part of the ROS build farm.  Do not install ros-kinetic-realsense-camera or ros-kinetic-librealsense, otherwise the steps above will not work correctly. 

## Support for non-Joule systems

The current public repo for RealSense does not contain uvcvideo-realsense-dkms package to support ZR300 camera.  This will be fixed soon, but in the meantime here is the workaround:
```bash
$ sudo apt instal dkms
$ wget http://jenkinsperc01.iil.intel.com:8080/userContent/freight-validation-candidates/pool/xenial/main/u/uvcvideo-realsense/uvcvideo-realsense-dkms_4.4.0-0ubuntu3%7E110.gbp24ae2c_all.deb
$ sudo dpkg -i uvcvideo-realsense-dkms_4.4.0-0ubuntu3~110.gbp24ae2c_all.deb 
```

Then unplug and replug ZR300.

## Usage Instructions

### Run SLAM

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

## Additional Information
For additional information about this repository, see https://wiki.ith.intel.com/display/perceptual/Guidelines+for+RealSense+SDK+ROS+Samples

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
