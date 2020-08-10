# ROS2 Wrapper for Intel&reg; RealSense&trade; Devices
These are packages for using Intel RealSense cameras (D400 series SR300 camera and T265 Tracking Module) with ROS2.

LibRealSense supported version: v2.36.0 (see [realsense2_camera release notes](https://github.com/IntelRealSense/realsense-ros/releases))

# NOTICE:
The building of this version is still underway. 
It is based on the the ROS1 version.
As a wrapper for a different environment it bears no restrictions where interface to clients is concerned.
Therefor, any comments as to incorrect or prefered topic names, parameters, usage or missed conventions are now very much welcome.

## Installation Instructions
This version supports ROS2 eloquent on Ubuntu 18.04.

   ### Step 1: Install the ROS2 distribution
   - #### Install [ROS2 Eloquent](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/), on Ubuntu 18.04.

   ```bash
  ROS_DISTRO=eloquent
  sudo apt update && sudo apt install curl gnupg2 lsb-release
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
  sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
  sudo apt update
  sudo apt install ros-$ROS_DISTRO-ros-base

  #Environment setup
  echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
  source ~/.bashrc

  sudo apt install python3-colcon-common-extensions -y

  #Install argcomplete (optional)
  sudo apt install python3-argcomplete
   ```


  ### Step 2: Install librealsense2:
   ### Install the latest Intel&reg; RealSense&trade; SDK 2.0
   - #### Install from [Debian Package](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages) - In that case treat yourself as a developer. Make sure you follow the instructions to also install librealsense2-dev and librealsense-dkms packages.

   #### OR
   - #### Build from sources by downloading the latest [Intel&reg; RealSense&trade; SDK 2.0](https://github.com/IntelRealSense/librealsense/releases/tag/v2.37.0) and follow the instructions under [Linux Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)


   ### Step 3: Install Intel&reg; RealSense&trade; ROS2 wrapper from Sources
   - Create a ROS2 workspace
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src/
   ```
   - Clone the latest Eloquent Intel&reg; RealSense&trade;  wrapper from [here](https://github.com/IntelRealSense/realsense-ros.git) into '~/ros2_ws/src/'
   ```bashrc
   git clone https://github.com/IntelRealSense/realsense-ros.git -b eloquent
   cd ~/ros2_ws
   ```

  ### Step 4: Install dependencies:
   ```bash
  sudo apt-get install python-rosdep -y
  sudo rosdep init
  rosdep update
  rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
  ```

  ### Step 5: Build:
  ```bash
  colcon build
  ```

  ### Step 6: Source (on each new terminal):
  ```bash
  . install/local_setup.bash
  ```


## Usage Instructions

### Start the camera node
To start the camera node in ROS:

```bash
  ros2 run realsense2_node realsense2_node 
```
or, with parameters, for example - pointcloud enabled:
```bash
ros2 run realsense2_node realsense2_node --ros-args -p filters:=pointcloud
```

This will stream all camera sensors and publish on the appropriate ROS topics.

## Known Issues
* Reconfigure parameters on runtime is missing.
* Topic names miss node namespace.
* No support for T265.
* Missing descriptive files (realsense2_description package).
* ROS2 optimizations

## License
Copyright 2018 Intel Corporation

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
