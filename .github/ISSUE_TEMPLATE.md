### System Configuration
Please complete Your Configuration detail below.

| Version          | Known Good   | Your Configuration |
| ---------------- | ------------:| ------------------:|
| Operating System | ubuntu 14.04 |                ??? |
| Kernel Version   |     4.4.0-13 |            ?.?.?-? |
| R200 FW          |    1.0.72.06 |            ?.?.?.? |
| librealsense     |        0.9.1 |              ?.?.? |

---
#### How to collect Configuration Data
*This section can be delete before submission.*

| Version          | Method |
| ---------------- | ------------ |
| Operating System | `cat /etc/*elease*` |
| Kernel Version   |    `uname -r` |
| R200 FW          |    View the ROS log from running node **OR** `<path to librealsense>/librealsense/bin/cpp-enumerate-devices | grep -i firmware` |
| librealsense     |      `cat <path to librealsense>/librealsense/readme.md | grep release-image | awk -F- '{print $3}'` |
---


### Expected Behavior


### Actual Behavior


### Steps to Reproduce


