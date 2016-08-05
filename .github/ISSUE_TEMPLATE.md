### System Configuration
Please complete Your Configuration detail below. Refer to the BKC at [Configuration](../realsense_camera/README.md#configuration).

| Version               | Your Configuration   |
|:--------------------- |:---------------------|
| Operating System      | ? ? ?                |
| Kernel                | ?-?-?                |
| Backend               | ?                    |
| ROS                   | ?                    |
| ROS RealSense         | ?.?.?                |
| librealsense          | ?.?.?                |
| Camera Type-Firmware  | ?-?.?.?.?            |

---
#### How to collect Configuration Data
*This section can be deleted before submission.*

| Version               | Method |
|:--------------------- |:------------ |
| Operating System      | `grep DISTRIB_DESCRIPTION /etc/*elease*` |
| Kernel                | `uname -r` |
| Backend               | `ls /sys/class | grep video4linux` |
| ROS                   | `rosversion -d` |
| ROS RealSense         | `rosversion realsense_camera` |
| librealsense          | `cat <path to librealsense>/librealsense/readme.md | grep release-image | awk -F- '{print $3}'` |
| Camera Type-Firmware  | View the ROS log from running nodelet |

---


### Expected Behavior


### Actual Behavior


### Steps to Reproduce


