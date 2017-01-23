realsense_pt_demo released by Perc-China(draft)
======================================
This package contains a ROS wrapper for Intel's realsense_persontracking library. The realsense_pt_demo package provides an image-based strategy to track and recognize the people in front of the camera as a ROS nodelet. For instance, it's guaranteed only with the RealSense device (tested with Depth and Color images received from ROS nodelet realsense_camera). The objective of this module is to get the information(can be pose, orientation, id etc.) of a person( or a specific person) in the images.  

1. Hardwarei/Software Requirement
--------------------------------------
### ubuntu requirements:
<br /\>     Ubuntu 16.04
<br /\>     gcc 4.9.3
### libraries of intel used(version beta3 RC5):
<br /\>     librealsense
<br /\>     Link: https://securewiki.ith.intel.com/pages/viewpage.action?pageId=510951805
### ros package requirements:
<br /\>     realsense_msgs
<br /\>     realsense_srvs
2. Example 
--------------------------------------
<br /\>    To use realsense_pt_demo, you need a RealSense device (LR200 or ZR300) and realsense_camera (RealSense ros package) in your system and launched.
<br /\>    To start realsense_pt_demo and realsense_camera with command: 
           roslaunch realsense_pt_demo camera_track.launch
3. Nodelet
--------------------------------------
##  3.1 realsense_pt_demo 
    The realsense_pt_demo nodelet take camera infos(sensor_msgs/CameraInfo), color stream, depth stream (sensor_msgs/Image), published by realsense_camera nodelet, send the information of person detected after calculation as realsense_msgs/Frame messages.
###  3.1.1 Subscribed Topics
      camera/depth/camera_info (sensor_msgs/CameraInfo)
        The intrinsic (and extrinsics) of camera (depth and depth to color)
      camera/color/camera_info (sensor_msgs/CameraInfo)
        The intrinsic of camera (color)
      camera/depth/image_raw (sensor_msgs/Image)
        Depth image stream received from RealSense device
      camera/color/image_raw (sensor_msgs/Image)
        Color image stream received from RealSense device
###  3.1.2 Published Topics
      person_tracking/person_tracking_output (realsense_msgs/Frame)
        Results information of person tracked
      image_result (sensor_msgs/Image)
        Image result of person tracked, you may see the image with rviz or rqt_image_view
###  3.1.3 Services
      person_tracking/tracking_config (realsense_srvs/TrackingConfig)
        Reconfigure several settings for PersonTracking
      person_tracing/recognition_request (realsense_srvs/RecognitionRequest)
        Reconfigure to register or remove a person to recognition 
      person_tracking/tracking_request (realsense_srvs/TrackingRequest)
        Reconfigure to start or stop tracking
      person_tracking/save_recognition (realsense_srvs/SaveRecognitionDB)
        Save the recognition data base to file
      person_tracking/load_recognition (realsense_srvs/LoadRecognitionDB)
        Load the recognition data base from file
      person_tracking/recognition_image_request (realsense_srvs/RecognitionImgRequest)
        Register or remove a person to recognition by image
###  3.1.4 Parameters
      ~recognitionEnabled (bool, default: false)
        Enable/Disable recognition
      ~sceletonEnable (bool, default: false)
        Enable/Disable skeleton
      ~gesturesEnabled (bool, default: false)
        Enable/Disable gestures
      ~trackingEnabled (bool, default: false)
        Enable/Disable track person
      ~trackingMode  (int, default: 1)
        0: will start the MW in detection mode.
        1: will track first person in frame
      ~loadDb (bool, default: false)
        Load recognition data base from file specified at (~dbPath) parameter
      ~dbPath (string, default: 'package_path/db')
        Path of recognition data base
      ~headPositionEnabled (bool, default: false)
        Enable/Disable headPosition
      ~headBoundingBoxEnabled (bool, default: false)
        Enable/Disable headBoundingBox
      ~landmarksEnabled (bool, default: false)
        Enable/Disable landmarks
      ~allowImgRegistration(bool, default: false)
        true: Allowed only image registration by calling service /person_tracking/registration_image_request
        false: Normal mode or PersonTracking without registration by image
      ~serial_no: (str::string, default: '') 
        The serial_number of device that you want to connect. By default, the first device will be used.
   
