ROS Node for Intel® RealSense™ Object Library
=========================================
This package contains a ROS wrapper for Intel's realsense library. The realsense_object package provides a solution to recognize, localize and track objects as a ROS nodelet.

## Nodelets

### realsense_ros_object_localization 
The realsense_localization nodelet takes color image and depth image from device, then processes localization on the frame, and publishes the result as realsense_or_msgs::ObjectsInBoxes message.

ObjectsInBoxes conteains color image header and array of ObjectInBoxs, which contains object (class name and probability), rect and 3d location.

#### Subscribed Topics
    camera/color/camera_info (sensor_msgs/CameraInfo)
	    The intrinsic of color camera
    camera/depth/camera_info (sensor_msgs/CameraInfo)
	    The intrinsic (and extrinsic) of camera( depth and depth to color)
    camera/color/image_raw (sensor_msgs::Image)
	    color image from camera
    camera/depth/image_raw (sensor_msgs::Image)
	    depth image from camera

#### Published Topics
    realsense_or_msgs::ObjectsInBoxes
	    color image header and array of ObjectInBoxs, which contains object name, rect and 3d location.
    realsense_or_msgs::cpu_gpu
        error message when unable to use GPU, and use CPU instead

#### Parameters
    ~show_rgb: (boolean, default: false) 
        if true - view localization result, drawn on rgb frame 
    ~confidence: (float, default: 0.7)
	    confidence threshold of localization result. result with confidence that low then the confidence will not be published.
    ~estimate_center: (boolean, default: true) 
	    if true - object estimation center will be enabled, and 3d location field in ObjectInBox will store it.
    ~use_CPU: (boolean, default: true) 
	    if true - use CPU when processing localization. otherwize - use GPU opencl.

### realsense_ros_object_recognition 
The realsense_recognition nodelet takes color image and depth image from device, then processes recognition on the frame, and publishes the result as realsense_or_msgs::ObjectArray message.

ObjectArray is an array of objects - class name and probability

#### Subscribed Topics
    camera/color/camera_info (sensor_msgs/CameraInfo)   
        The intrinsic of color camera
    camera/depth/camera_info (sensor_msgs/CameraInfo)
	    The intrinsic (and extrinsic) of camera( depth and depth to color)
    camera/color/image_raw (sensor_msgs::Image)
	    color image from camera
    camera/depth/image_raw (sensor_msgs::Image)
	    depth image from camera

#### Published Topics
    realsense_or_msgs::ObjectArray
        array of objects - class name and probability
        
#### Command Line Parameters
    ~show_rgb to show recognition result with rgb frame

### realsense_ros_object_tracking
The realsense_tracking nodelet takes color image and depth image from device, get ROIs of objects to track, and then processes tracking on the frame, and publishes the result as realsense_or_msgs::TrackedObjectsArray message.

TrackedObjectsArray conteains color image header and array of TrackedObject, which contains object index, rect and 3d location.

#### Subscribed Topics
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

#### Published Topics
    realsense_or_msgs::TrackedObjectsArray
	    color image header and array of TrackedObject, which contains object index, rect and 3d location.

#### Parameters
    ~show_rgb: (boolean, default: false) 
	    if true - view localization result, drawn on rgb frame 
    ~estimate_center: (boolean, default: true) 
	    if true - object estimation center will be enabled, and 3d location field in ObjectInBox will store it.
    ~use_CPU: (boolean, default: true) 
	    if true - use GPU opencl when processing localization. otherwize - use CPU.
    ~max_number_of_objects: (int, default: 5)
	    number of object to track. if the input roi array size is higher - tracking will be processed only on first max_number_of_objects ROIs

## Usage

Object package consists of 3 nodelets: realsense_ros_object_localization, realsense_ros_object_recognition, and realsense_ros_object_tracking.

The user should create his combination from those 3 nodelets by creating new nodelet that associates with them.


### For sample app

#### From camera
```bash
$  roslaunch realsense_ros_object demo_object.launch
```

#### From ros bag file

Recorging a bag file:
```bash
$  roslaunch realsense_ros_object record_bag_object.launch bag_path:=mypath
```
Running object demo from bag:
```bash
$  roslaunch realsense_ros_object demo_object_from_bag.launch bag_path:=mypath
```

The sample app demonstrates a combination of localization and tracking: process localization when the user hint space key, and the ROIs of localization output are the input to tracking.

## Testing

The object package can be tested with pre-recorded data using the provided ROS unit test.  No physical camera needs to be present in order to run the test.  The following steps can be used to build the unit test and download the pre-recorded ROS .bag data:

```bash
$ cd ~/catkin_ws
$ catkin_make -DREALSENSE_ENABLE_TESTING=On
$ rostest realsense_ros_object object.test
```

You will see the test execute with the graphics display recognizing known objects, and the test passes with a "RESULT: SUCCESS" status.

**Note:** Depending on your internet connection speed, enabling 'REALSENSE_ENABLE_TESTING' can cause catkin_make to run for very long time (more than 5 minutes), as it downloads required pre-recorded .bag data.
	
