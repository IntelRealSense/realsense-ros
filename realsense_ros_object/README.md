ROS Node for Intel® RealSense™ Object Library
=========================================

(Shoshana, your AR: Update this documentation)

# (Old Content) realsense_object released by Perc-OR(draft)

This package contains a ROS wrapper for Intel's realsense library. The realsense_object package provides a solution to recognize, localize and track objects as a ROS nodelet.

1. Hardwarei/Software Requirement
----------------------------------------------
  To use realsense_object, you need an agent with a RealSense camera mounted.
###  ubuntu requirements:
<br /\>     Ubuntu 16.04
<br /\>     gcc 4.9.3
<br /\>     libglfw3-dev
<br /\>     opencv 3.1 as system opencv (https://sourceforge.net/projects/opencvlibrary/files/opencv-unix/3.1.0/opencv-3.1.0.zip/download)
###  libraries of intel used(version beta3 RC5):
<br /\>     Linux SDK
<br /\>     librealsense
<br /\>     Link: https://securewiki.ith.intel.com/pages/viewpage.action?pageId=510951805
2. Example 
---------------------------------------
<br /\>    To start realsense_localization with command: 
<br /\>    roslaunch realsense_localization realsense_localization_launch.launch 
<br /\>    To start realsense_recognition with command: 
<br /\>    roslaunch realsense_recognition realsense_recognition_launch.launch 
<br /\>    To start realsense_tracking with command: 
<br /\>    roslaunch realsense_tracking realsense_tracking_launch.launch 
<br /\>    To start realsense_ormgr with command: 
<br /\>    roslaunch realsense_ormgr realsense_ormgr_launch.launch 
<br /\>    To start realsense_orview with command: 
<br /\>    roslaunch realsense_orview realsense_orview_launch.launch 
3. Nodelet
--------------------------------------------
## 3.1 realsense_localization 
    The realsense_localization nodelet takes color image and depth image from device, then processes localization on the frame, and publishes the result as realsense_or_msgs::ObjectsInBoxes message.
    ObjectsInBoxes conteains color image header and array of ObjectInBoxs, which contains object (class name and probability), rect and 3d location.
###  3.1.1 Subscribed Topics
      camera/color/camera_info (sensor_msgs/CameraInfo)
        The intrinsic of color camera
      camera/depth/camera_info (sensor_msgs/CameraInfo)
        The intrinsic (and extrinsic) of camera( depth and depth to color)
      camera/color/image_raw (sensor_msgs::Image)
        color image from camera
      camera/depth/image_raw (sensor_msgs::Image)
        depth image from camera
###  3.1.2 Published Topics
      realsense_or_msgs::ObjectsInBoxes
          color image header and array of ObjectInBoxs, which contains object name, rect and 3d location.
      realsense_or_msgs::cpu_gpu
          error message when unable to use GPU, and use CPU instead
###  3.1.3 Parameters
      ~show_rgb: (boolean, default: false) 
          if true - view localization result, drawn on rgb frame 
      ~confidence: (flaot, default: 0.7)
          confidence threshold of localization result. result with confidence that low then the confidence will not be published.
      ~estimate_center: (boolean, default: true) 
          if true - object estimation center will be enabled, and 3d location field in ObjectInBox will store it.
      ~use_CPU: (boolean, default: true) 
          if true - use GPU opencl when processing localization. otherwize - use CPU.
## 3.2 realsense_recognition 
    The realsense_recognition nodelet takes color image and depth image from device, then processes recognition on the frame, and publishes the result as realsense_or_msgs::ObjectArray message.
    ObjectArray is an array of objects - class name and probability
###  3.2.1 Subscribed Topics
      camera/color/camera_info (sensor_msgs/CameraInfo)
        The intrinsic of color camera
      camera/depth/camera_info (sensor_msgs/CameraInfo)
        The intrinsic (and extrinsic) of camera( depth and depth to color)
      camera/color/image_raw (sensor_msgs::Image)
        color image from camera
      camera/depth/image_raw (sensor_msgs::Image)
        depth image from camera
###  3.2.2 Published Topics
      realsense_or_msgs::ObjectArray
          array of objects - class name and probability
###  3.2.3 Command Line Parameters
      ~show_rgb to show recognition result with rgb frame
## 3.3 realsense_tracking
    The realsense_tracking nodelet takes color image and depth image from device, get ROIs of objects to track, and then processes tracking on the frame, and publishes the result as realsense_or_msgs::TrackedObjectsArray message.
    TrackedObjectsArray conteains color image header and array of TrackedObject, which contains object index, rect and 3d location.
###  3.3.1 Subscribed Topics
      camera/color/camera_info (sensor_msgs/CameraInfo)
        The intrinsic of color camera
      camera/depth/camera_info (sensor_msgs/CameraInfo)
        The intrinsic (and extrinsic) of camera( depth and depth to color)
      camera/color/image_raw (sensor_msgs::Image)
        color image from camera
      camera/depth/image_raw (sensor_msgs::Image)
        depth image from camera
      realsense/localized_objects (realsense_or_msgs::ObjectsInBoxes)
        localization result. for tracking + localization, when localization result is used as tracking ROIs input.
      realsense/objects_to_track (realsense_or_msgs::TrackedObjectsArray)
        tracking ROIs input. for tracking only.
###  3.3.2 Published Topics
      realsense_or_msgs::TrackedObjectsArray
          color image header and array of TrackedObject, which contains object index, rect and 3d location.
###  3.3.3 Parameters
      ~show_rgb: (boolean, default: false) 
          if true - view localization result, drawn on rgb frame 
      ~estimate_center: (boolean, default: true) 
          if true - object estimation center will be enabled, and 3d location field in ObjectInBox will store it.
      ~use_CPU: (boolean, default: true) 
          if true - use GPU opencl when processing localization. otherwize - use CPU.
      ~max_number_of_objects: (int, default: 5)
          number of object to track. if the input roi array size is higher - tracking will be proccessed only on first max_number_of_objects ROIs
## 3.4 realsense_ormgr
    manager, sample app to show how to manipulate localization and tracking. process localization when get realsense_or_msgs::UI message.
    
## 3.5 realsense_orview
    viewer, sample app to show how to use realsense_ormgr. process localization when the user hint space key.
    
4. run sample app
--------------------------------------------
<br /\>    launch realsense_ros_camera
<br /\>     
<br /\>    for each nodelet open a new tab (Ctrl+shift+t) and type
<br /\>      source devel/setup.bash
<br /\>    
<br /\>    window 2
<br /\>      roslaunch realsense_localization realsense_localization_launch.launch
<br /\>    
<br /\>    window 3
<br /\>      roslaunch realsense_tracking realsense_tracking_launch.launch
<br /\>    
<br /\>    window 4
<br /\>      roslaunch realsense_ormgr realsense_ormgr_launch.launch
<br /\>    
<br /\>    window 5
<br /\>      roslaunch realsense_orview realsense_orview_launch.launch
