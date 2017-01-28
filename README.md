There is the draft version ROS packages released by Perc-china.

###Libraries used:(version beta3_RC5_22_12_2016_ww52)

<br /\>    SLAM: libreansense_slam.so. 

<br /\>    PT: librealsense_persontracking.so. 

<br /\>    CAMERA: librealsense.so. 

The release notes in the packages are draft version, they will be updated very soon. Thanks.
 
## Installation Instructions

See 'script_setup.sh' in this repo to install everything needed from scratch, from clean Ubuntu install
```bash
$ wget https://github.intel.com/raw/IntelRealSense/realsense_ros/master/script_setup.sh
$ chmod a+x ./script_setup.sh
$ ./script_setup.sh
```

**Note**: These instructions above currently use a different librealsense than provided publically as part of the ROS build farm.  Do not install ros-kinetic-realsense-camera or ros-kinetic-librealsense, otherwise the steps above will not work correctly. 

## Support for non-Joule systems

The current public repo for RealSense does not contain uvcvideo-realsense-dkms package to support ZR300 camera.  This will be fixed soon, but in the meantime here is the workaround:
```bash
$ sudo apt instal dkms
$ wget http://jenkinsperc01.iil.intel.com:8080/userContent/freight-validation-candidates/pool/xenial/main/u/uvcvideo-realsense/uvcvideo-realsense-dkms_4.4.0-0ubuntu3%7E110.gbp24ae2c_all.deb
$ sudo dpkg -i uvcvideo-realsense-dkms_4.4.0-0ubuntu3~110.gbp24ae2c_all.deb 
```

Then unplug and replug ZR300.

## Usage Instructions

### Run SLAM

To run the slam engine:
```bash
$ cd catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch rs_slam_test camera_slam_nodelet.launch
```

To see estimated pos, in another window:
```bash
$ cd catkin-ws
$ source devel/setup.bash
$ rostopic echo /pose2d
```

## Additional Information
For additional information about this repository, see https://wiki.ith.intel.com/display/perceptual/Guidelines+for+RealSense+SDK+ROS+Samples
