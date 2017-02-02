# Intel&reg; RealSense&trade; ROS Person
Person detection and tracking package designed for for use with Intel RealSense cameras under ROS.

http://www.intel.com/realsense

http://wiki.ros.org/RealSense

# Contributing to the Project

The Intel&reg; RealSense&trade; ROS Person Project is developed and distributed under
a BSD license as noted in [licenses/License.txt](licenses/License.txt).

By making a contribution to this project, I certify that:

(a) The contribution was created in whole or in part by me and I
have the right to submit it under the open source license
indicated in the file; or

(b) The contribution is based upon previous work that, to the best
of my knowledge, is covered under an appropriate open source
license and I have the right under that license to submit that
work with modifications, whether created in whole or in part
by me, under the same open source license (unless I am
permitted to submit under a different license), as indicated
in the file; or

(c) The contribution was provided directly to me by some other
person who certified (a), (b) or (c) and I have not modified
it.

(d) I understand and agree that this project and the contribution
are public and that a record of the contribution (including all
personal information I submit with it, including my sign-off) is
maintained indefinitely and may be redistributed consistent with
this project or the open source license(s) involved.

#Alpha Release

##Configuration:
| Version        | Best Known (kinetic) |
|:-------------- |:---------------------|
| OS             | Ubuntu 16.04 LTS     |
| Kernel         | 4.4.0-45-generic     |
| librealsense_persontracking | Beta3   |

##Building this Package from Source:
<b>Note:</b> ROS beginners, please ensure that the basic ROS environment and catkin workspace has been set up by following the instructions at http://wiki.ros.org/ROS/Installation.

<b>Prerequisite:</b> Successful build and use of this ROS package assumes you have already installed the Person-Tracking Middleware.

Install the package and its dependent packages as follows:
 - Clone the repo in the `src` directory of your catkin workspace.
 - Install the dependent packages using the command `rosdep install --from-paths src/realsense_person`.
 - Build the package using the command `catkin_make`.
 - Validate the installation by running one of the launch files.

##Package Features:
###Launch files
`person_sample.launch`

   Launches the ROS R200 nodelet and the Person nodelet.

   <b>Note:</b> This is a sample using the RealSense <b>R200</b> camera.
   If you are using any other RealSense camera that is supported by the `realsense_camera` package,
   update the `camera_type` argument with the corresponding value.

###Subscribed Topics
`color/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))

`depth/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))

`depth/image_raw` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

`color/image_raw` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

###Published Topics
`detection_data` ([realsense_person/PersonDetection](msg/PersonDetection.msg))

   Contains the basic information of all the people detected in a frame.
   If a person is being tracked, it will return the information of only that person.

`detection_image` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

   Contains the "detection_data" overlayed on the corresponding color image frame.

`tracking_data` ([realsense_person/PersonTracking](msg/PersonTracking.msg))

   Contains detailed information of the person being tracked.
   <b>Note:</b> Use the "start_tracking_person" service to start tracking a person. If a person is not being tracked,
   this topic will not be published.

   The information contained in this topic can be controlled using the parameters
   "enable_head_bounding_box", "enable_face_landmarks", "enable_gestures"
   and "enable_skeleton_joints". Due to performace implications, only some of these parameters are enabled by default.

   <b>Note:</b> Enable these parameters on a "need-to" basis as they are compute intensive and may lead to unstable data.
   Refer to the <b>Dynamic Parameters</b> section below for details regarding these parameters.

`tracking_image` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

   Contains the "tracking_data" overlayed on the corresponding color image frame.
   Refer to the "tracking_data" topic for more details.

###Services
`get_tracking_state` ([realsense_person/GetTrackingState](srv/GetTrackingState.srv))

   Returns the tracking state. If nobody is being tracked, it will return the list of tracking_ids of all the detected people.
   If a person is being tracked, it will return the tracking_id of just that person.

`register_person` ([realsense_person/Register](srv/Register.srv))

   Assigns a unique recognition id for the specified person and saves the person's descriptor features.

`recognize_person` ([realsense_person/Recognize](srv/Recognize.srv))

   Returns the recognition id if the specified person has been registered.

`reinforce_person` ([realsense_person/Reinforce](srv/Reinforce.srv))

   Saves a new set of descriptor features for the specified person.

`start_tracking_person` ([realsense_person/StartTracking](srv/StartTracking.srv))

   Enables the tracking feature in the middleware and starts tracking a person based on tracking_id.
   This service must be called in order to generate data for the "tracking" related topics.

`stop_tracking` ([realsense_person/StopTracking](srv/StopTracking.srv))

   Stops tracking and disables the tracking feature in the middleware.

###Custom Messages
Refer to the defintion at ([Custom Messages](msg/)).

###Static Parameters
`subscribe_rate` (default: 30)

   Controls the subscription rate of the color and depth topics.

###Dynamic Parameters
`detection_rate` (default: 30)

   Controls the publish rate of the detection topics.

`tracking_rate` (default: 30)

   Controls the publish rate of the tracking topics.

`enable_recognition` (default: true)

   Enables the "recognition" feature in the middleware.

`enable_head_bounding_box` (default: false)

   Enables the "head bounding box" feature in the middleware.

`enable_face_landmarks` (default: false)

   Enables the "face landmarks" feature in the middleware.

`enable_gestures` (default: false)

   Enables the "gesture" feature in the middleware.

   <b>Note:</b> The "gesture" feature is currently in development phase and not fully stable.

`enable_skeleton_joints` (default: false)

   Enables the "skeleton joints" feature in the middleware.

###Coordinates and Units
The "image" points are pixel points where origin (0,0) is the top-left corner of the image
with positive x-axis pointing to right and positive y-axis pointing down.

The "world" points are points in meters where the origin (0,0) is the center of the color camera
with positive x-axis pointing to right, positive y-axis pointing down and positive z-axis pointing forward,
with respsect to the camera's view.

###Unit Tests
Refer to [Unit Tests](test/) for the `.test` files.

Run the unit tests using the command `rostest realsense_person <test file>`.

