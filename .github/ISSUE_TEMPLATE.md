### System Configuration
Please complete Your Configuration detail below. Refer to the BKC at [Configuration](../realsense_camera/README.md#configuration).

| Version          | Your Configuration   |
|:---------------- |:---------------------|
| Operating System | ? ? ?                |
| Kernel           | ?-?-?                |
| Backend          | ?                    |
| ROS              | ?                    |
| ROS RealSense    | ?.?.?                |
| librealsense     | ?.?.?                |
| R200 Firmware    | ?.?.?.?              |

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


