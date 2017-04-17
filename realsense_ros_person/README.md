# ROS Node for Intel® RealSense Person Library
=========================================

This package contains a ROS wrapper for Intel's Person library.
The realsense_ros_person package provides a image-based stategy to follow the 
people in front of the camera as a ROS nodelet. The objective of this module is
to find out the information (can be pose, orientation, id etc.) of a person (or a specific person)
in the images.

## Nodelets

### realsense_ros_person - person nodelet - wraps Person API
    The realsense_ros_person nodelet takes sensor_msgs/CameraInfo camera infos and sensor_msgs/Image images(color, depth),then send the informations of person detected as realsense_ros_person/Frame realsense_ros_person/FrameTest messages.

#### Subscribed Topics
    camera/depth/camera_info (sensor_msgs/CameraInfo)
        The informations of camera used
    camera/color/camera_info (sensor_msgs/CameraInfo)
        The informations of camera used
    camera/depth/image_raw (sensor_msgs/Image)
        Depth image got from realsense camera
    camera/color/image_raw (sensor_msgs/Image)
        Color image got from realsense camera

#### Published Topics
    person_tracking_output (realsense_ros_person/Frame) Informations of person tracked
    person_tracking_output_test (realsense_ros_person/FrameTest) Information of person tracked + color image from camera
    /person_tracking/module_state (realsense_ros_person/PersonModuleState) Person tracking module state (which features is enabled tracking state, etc.)

#### Services
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

#### Parameters
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

### realsense_ros_person_sample_nodelet - person sample node - controls person node and visualizes output

#### Subscribed Topics
   person_tracking/person_tracking_output_test (realsense_pt_msgs/FrameTest)

#### Published Topics
    none

#### Services
    none

#### GUI commands
    Start tracking:
        Mouse wheel click
    Register:
       Mouse left button click
    Recognize:
       Ctrl+left mouse button click

#### Parameters
IMPORTANT this parameters will overide parameters of realsense_ros_person nodelet,
configuration request will be sent to realsense_ros_person nodelet

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

## Usage
The person package consists of 2 nodelets:

1. realsense_ros_person nodelet is the Person API wrapper for ROS.
2. realsense_ros_person_sample_nodelet - demo for usage of Person ROS API - visualize output of realsense_ros_person nodelet, provide GUI for realsense_ros_person node control(Start tracking, Register/Recognize users)

The person node by default starts in detection mode, and some features requires tracking mode (e.g. skeleton).

### For person tracking feature
```bash
$ roslaunch realsense_ros_person demo_person_tracking.launch
```

### For person gestures feature(pointing & wave)
```bash
$ roslaunch realsense_ros_person demo_person_gestures.launch
```

### For skeleton feature
```bash
$ roslaunch realsense_ros_person demo_person_skeleton.launch
```

### For person face features (recogntion, landmarks, head pose, head bounding box)
```bash
$ roslaunch realsense_ros_person demo_person_face_features.launch
  ```


### Recorging a bag file:
```bash
$  roslaunch realsense_ros_person record_bag_person.launch bag_path:=mypath
```
### Running person demo from bag:
```bash
$  roslaunch realsense_ros_person demo_person_tracking_from_bag.launch bag_path:=mypath
```

## Testing

The person package can be tested with pre-recorded data using the provided ROS unit test.  No physical camera needs to be present in order to run the test.  The following steps can be used to build the unit test and download the pre-recorded ROS .bag data:

```bash
$ cd ~/catkin_ws
$ catkin_make -DREALSENSE_ENABLE_TESTING=On
$ rostest realsense_ros_person person_detection.test
$ rostest realsense_ros_person recognition.test
$ rostest realsense_ros_person wave_detection.test
```

You will see the three tests each execute with command line output only, and then each test passes with a "RESULT: SUCCESS" status.

**Note:** Depending on your internet connection speed, enabling 'REALSENSE_ENABLE_TESTING' can cause catkin_make to run for very long time (more than 5 minutes), as it downloads required pre-recorded .bag data.

# Person features requirements for tracking mode
Part of person features works only at tracking/detection mode.

|Feature name           |Tracking mode          |
|-----------------------|-----------------------|
|Tracking               |Tracking               |
|Skeleton               |Tracking               |
|Pointing gesture       |Tracking               |
|Wave gesture           |Detection              |
|Recognition            |(either)               |
|Landmarks              |(either)               |
|Head bounding box      |(either)               |
|Head pose              |(either)               |
