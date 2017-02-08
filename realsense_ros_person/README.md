realsense_ros_person v0.0.1

This package contains a ROS wrapper for Intel's PersonTracking library. The realsense_pt package provides a image-based stategy to follow the people in front of the camera as a ROS nodelet. For instance,it's garantieed only with RealSense ZR300(tested with Depth and Color images recieved from ROS nodelet realsense_camera). The objectif of this module is to find out the informations(can be pose,oriantation,id etc.) of a person(or a specific person) in the images.

1. Hardware/Software Requirement

  system requirements:
    Ubuntu 16.04 LTS
    GTK+2.0
    libjpeg-dev
    libpng-dev
    librealsense 2.2.0
    realsense-sdk
    person tracking
    ROS indigo

2. Example 
   To use realsense_ros_person,you may need a RealSense camera (ZR300 minimum) with the realsense_camera package in your system and launched.  

  2.1 Quick start
    To start realsense camera and realsense_pt nodes with command:
        roslaunch realsense_pt cam_track.launch
    To start realsense_pt only with command:
        roslaunch realsense_pt track.launch
    To start a graphic test application(compiled as part of project)
        devel/lib/realsense_pt_sample/realsense_pt_sample
    And you can set the configuration according to need, by modifier the params in
        ./realsense_pt/launch/pt_params.yaml
3. Nodelet
  3.1 realsense_pt - person tracking node - wraps person tracking API
    The realsense_pt nodelet takes sensor_msgs/CameraInfo camera infos and sensor_msgs/Image images(color, depth),then send the informations of person detected as realsense_pt_msgs/Frame realsense_pt_msgs/FrameTest messages.
  3.1.1 Subscribed Topics
    camera/depth/camera_info (sensor_msgs/CameraInfo)
        The informations of camera used
    camera/color/camera_info (sensor_msgs/CameraInfo)
        The informations of camera used
    camera/depth/image_view (sensor_msgs/Image)
        Depth image got from realsense camera
    camera/color/image_view (sensor_msgs/Image)
        Color image got from realsense camera
  3.1.2 Published Topics
    person_tracking/person_tracking_output (realsense_pt_msgs/Frame) Informations of person tracked
    person_tracking/person_tracking_output_test (realsense_pt_msgs/FrameTest) Informations of person tracked + color image from camera

  3.1.3 Services
    person_tracking/tracking_config (realsense_srvs/TrackingConfig)
        Reconfigure several settings for PersonTracking
    person_tracking/register_request (realsense_srvs/RecognitionRegister)
        Register person at recognition database
    person_tracking/recognition_request (realsense_srvs/Recognition)
        Recognize person
    person_tracking/tracking_request (realsense_srvs/TrackingRequest)
        Reconfigure to start or stop tracking
    person_tracking/save_recognition (realsense_srvs/SaveRecognitionDB)
        Save the recognition data base to file
    person_tracking/load_recognition (realsense_srvs/LoadRecognitionDB)
        Load the recognition data base from file

  3.1.4 Parameters
    ~recognitionEnabled (bool, default: true)
        Enable/Disable recognition
    ~sceletonEnabled (bool, default: true)
        Enable/Disable skeleton
    ~gesturesEnabled (bool, default: true)
        Enable/Disable gestures
    ~trackingEnabled (bool, default: true)
    ~headPoseEnabled (bool, default: true)
    ~headBoundingBoxEnabled (bool, default: true)
    ~landmarksEnabled (bool, default: true)
    ~loadDb (bool, default: false)
        Loads recognition data base from file specified at (~dbPath) parameter
    ~dbPath: 'package_path/db'
        Path of recognition data base
    ~isTestMode: (bool, defaultL false)
        Start node at test mode, publish FrameTest messages on person_tracking/person_tracking_output_test topic

   3.2.1 realsense_ros_person_sample Demo/test node - input: person tracking test output, output color image with person tracking data (rectangles etc.)
   3.2.1 Subscribed Topics
        person_tracking/person_tracking_output_test (realsense_pt_msgs/FrameTest)
   3.2.2 Published Topics
        nothing
   3.2.3 Services
        nothing
   3.2.4 GUI commands:
        Start tracking:
            Mouse wheel click
        Register:
            Mouse left button click
        Recognize:
            Ctrl+left mouse button click
   4. How to run:
        Person tracking node only (assumes that camera node already run):
            roslaunch realsense_ros_person track.launch
        Person tracking node and camera node:
            roslaunch realsense_ros_person cam_track.launch
        Person tracking node and camera node and person tracking test(sample) node:
            roslaunch realsense_ros_person realsense_person_tracking_test.launched
