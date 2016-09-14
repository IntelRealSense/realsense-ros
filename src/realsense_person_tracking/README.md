Person tracking ROS node

# Requirements
* librealsense v1.10.0
* realsense_sdk v0.2.10
* realsense_persontracking 0.3.5

# Person tracking node limitations
* color image format  RGB

# How to run
1. run camera node
2. rosrun realsense_person_tracking_test realsense_person_tracking_test _skeleton:=true _segmentation:=0 _trackingMode:=1 _gestures:=true
3. rosrun realsense_person_tracking realsense_person_tracking 
