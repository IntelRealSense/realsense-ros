### System Configuration
Please complete Your Configuration detail below.

| Version          | Best Known           | Your Configuration   |
|:---------------- |:---------------------|:---------------------|
| Operating System | Ubuntu 14.04.4 LTS   | ? ? ?                |
| Kernel           | 4.4.0-040400-generic | ?-?-?                |
| Backend          | video4linux          | ?                    |
| ROS              | indigo               | ?                    |
| ROS RealSense    | Latest Release       | ?.?.?                |
| librealsense     | 0.9.1                | ?.?.?                |
| R200 Firmware    | 1.0.72.06            | ?.?.?.?              |

---
#### How to collect Configuration Data
*This section can be deleted before submission.*

| Version          | Method |
|:---------------- |:------------ |
| Operating System | `grep DISTRIB_DESCRIPTION /etc/*elease*` |
| Kernel           | `uname -r` |
| Backend          | `ls /sys/class | grep video4linux` |
| ROS              | `rosversion -d` |
| ROS RealSense    | `rosversion realsense_camera` |
| librealsense     | `cat <path to librealsense>/librealsense/readme.md | grep release-image | awk -F- '{print $3}'` |
| R200 Firmware    | View the ROS log from running nodelet **OR** `<path to librealsense>/librealsense/bin/cpp-enumerate-devices | grep -i firmware` |

---


### Expected Behavior


### Actual Behavior


### Steps to Reproduce


