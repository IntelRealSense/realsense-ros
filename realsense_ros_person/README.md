# ROS Node for Intel® RealSense Person Library
=========================================

This package contains a ROS wrapper for Intel's Person library.
The realsense_ros_person package provides a image-based stategy to follow the 
people in front of the camera as a ROS nodelet. The objective of this module is
to find out the information (can be pose, orientation, id etc.) of a person (or a specific person)
in the images.

## 1. Design and Usage
The person package consists of 2 nodelets:

1. realsense_ros_person nodelet is the Person API wrapper for ROS.
2. realsense_ros_person_sample_nodelet - demo for usage of Person ROS API - visualize output of realsense_ros_person nodelet, provide GUI for realsense_ros_person node control(Start tracking, Register/Recognize users)
    
## 2. Run person nodes (Person API wrapper + sample):
Person node starts in detection mode, some features requries tracking mode(e.g. skeleton).

Aditional information:

* [Person features tracking mode requirements](#4-person-features-requirements-for-tracking-mode)

* [Person sample nodelet GUI commands](#324-gui-commands)

### For person tracking feature
    roslaunch realsense_ros_person realsense_person_demo_tracking.launch
### For person gestures feature(pointing & wave)
    roslaunch realsense_ros_person realsense_person_demo_gestures.launch
### For skeleton feature
    roslaunch realsense_ros_person realsense_person_demo_skeleton.launch
### For person face features (recogntion, landmarks, head pose, head bounding box)
    roslaunch realsense_ros_person realsense_person_demo_face_features.launch
    
## 3. Nodelet
### 3.1 realsense_ros_person - person nodelet - wraps Person API
    The realsense_ros_person nodelet takes sensor_msgs/CameraInfo camera infos and sensor_msgs/Image images(color, depth),then send the informations of person detected as realsense_ros_person/Frame realsense_ros_person/FrameTest messages.
####  3.1.1 Subscribed Topics
    camera/depth/camera_info (sensor_msgs/CameraInfo)
        The informations of camera used
    camera/color/camera_info (sensor_msgs/CameraInfo)
        The informations of camera used
    camera/depth/image_raw (sensor_msgs/Image)
        Depth image got from realsense camera
    camera/color/image_raw (sensor_msgs/Image)
        Color image got from realsense camera
#### 3.1.2 Published Topics
    person_tracking_output (realsense_ros_person/Frame) Informations of person tracked
    person_tracking_output_test (realsense_ros_person/FrameTest) Informations of person tracked + color image from camera

#### 3.1.3 Services
    person_tracking/tracking_config (realsense_ros_person/TrackingConfig)
        Reconfigure several settings for PersonTracking
    person_tracking/register_request (realsense_ros_person/RecognitionRegister)
        Register person at recognition database
    person_tracking/recognition_request (realsense_ros_person/Recognition)
        Recognize person
    person_tracking/start_tracking_request (realsense_ros_person/StartTracking)
        Start tracking on specific person
    person_tracking/stop_tracking_request (realsense_ros_person/StopTracking)
        Start tracking on specific person
    person_tracking/save_recognition (realsense_ros_person/SaveRecognitionDB)
        Save the recognition data base to file
    person_tracking/load_recognition (realsense_ros_person/LoadRecognitionDB)
        Load the recognition data base from file

#### 3.1.4 Parameters
    ~recognitionEnabled (bool, default: false)
        Enable/Disable recognition
    ~sceletonEnabled (bool, default: false)
        Enable/Disable skeleton
    ~pointingGestureEnabled (bool, default: false)
        Enable/Disable pointing gesture
    ~waveGestureEnabled
        Enable/Disable wave gesture
    ~trackingEnabled (bool, default: true)
        Enable/Disable tracking
    ~headPoseEnabled (bool, default: false)
        Enable/Disable head pose
    ~headBoundingBoxEnabled (bool, default: false)
        Enable/Disable head bounding box
    ~landmarksEnabled (bool, default: false)
        Enable/Disable face landmarks
    ~loadDb (bool, default: false)
        Loads recognition data base from file specified at (~dbPath) parameter (load as part of node initialization)
    ~dbPath: 'package_path/db'
        Path of recognition data base
    ~isTestMode: (bool, defaultL false)
        Start node at test mode, publish FrameTest messages on person_tracking/person_tracking_output_test topic

### 3.2 realsense_ros_person_sample_nodelet - person sample  node - controls person node and visualizes output
#### 3.2.1 Subscribed Topics
   person_tracking/person_tracking_output_test (realsense_pt_msgs/FrameTest)
#### 3.2.2 Published Topics
    nothing
#### 3.2.3 Services
    nothing
####  3.2.4 GUI commands
    Start tracking:
        Mouse wheel click
    Register:
       Mouse left button click
    Recognize:
       Ctrl+left mouse button click
#### 3.2.5 Parameters
IMPORTANT this parameters will overide parameters of realsense_ros_person nodelet,
configuration request will be send to realsense_ros_person nodelet

    ~recognitionEnabled (bool, default: false)
        Enable/Disable recognition
    ~sceletonEnabled (bool, default: false)
        Enable/Disable skeleton
    ~pointingGestureEnabled (bool, default: false)
        Enable/Disable pointing gesture
    ~waveGestureEnabled
        Enable/Disable wave gesture
    ~trackingEnabled (bool, default: true)
        Enable/Disable tracking
    ~headPoseEnabled (bool, default: false)
        Enable/Disable head pose
    ~headBoundingBoxEnabled (bool, default: false)
        Enable/Disable head bounding box
    ~landmarksEnabled (bool, default: false)
        Enable/Disable face landmarks

# 4 Person features requirements for tracking mode
Part of person features works only at tracking/detection mode.

|Feature name           |Tracking mode          |
|-----------------------|-----------------------|
|Tracking               |Tracking               |
|Skeleton               |Tracking               |
|Pointing gesture       |Tracking               |
|Wave gesture           |Detection              |
|Recognition            |Not important          |
|Landmarks              |Not important          |
|Head bounding box      |Not important          |
|Head pose              |Not important          |
