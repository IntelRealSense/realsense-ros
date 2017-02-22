ROS Node for Intel® RealSense™ Object Library
=========================================
This package contains a ROS wrapper for Intel's realsense library. The realsense_object package provides a solution to recognize, localize and track objects as a ROS nodelet.

1. Design and Usage
---------------------------------------
Object package consists of 3 nodelets: realsense_ros_object_localization, realsense_ros_object_recognition, and realsense_ros_object_tracking.
The user should create his combination from those 3 nodelets by creating new nodelet that assosiate with them.

2. Run sample app
--------------------------------------------
<br /\>    roslaunch realsense_ros_object demo_object.launch

usage: The sample app demonstrates a combination of localization and tracking: process localization when the user hint space key, and the ROIs of localization output are the input to tracking.
nodelets: realsense_ros_object_sample1, realsense_ros_object_viewer.

3. Nodelet
--------------------------------------------
## 3.1 realsense_ros_object_localization 
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
## 3.2 realsense_ros_object_recognition 
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
## 3.3 realsense_ros_object_tracking
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
    
