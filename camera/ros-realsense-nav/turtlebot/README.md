<link rel="stylesheet" href="style-doc.css" />

# Navigation with Intel&reg; RealSense&trade; **R200** camera
This document presents the transition from a **Kinect** to the **R200** for the ROS Navigation stack.
It assumes the **R200** drivers were already installed, and that the navigation using the Kinect is working.

#### Document outline
Click on the following links to navigate this document

- [Initial setup](#a-initial-setup)
- [Mapping](#b-mapping)
- [Navigation](#c-navigation)
- [Simulating a mapping](#d-simulating-a-mapping)

## A - Initial setup

First, make sure you have the latest versions of the ROS navigation stack. Specifically, check that the following packages are up-to-date:

- **turtlebot_description**
- **turtlebot_bringup**

Before being able to use the navigation stack with the **R200**, you need to set a few things up.

The `install_realsense_navigation.sh` script (located in the `install_resources` folder) will do everything needed. It will place the required files in their respective locations.

From now on, if you want to use the **R200** as your turtlebot 3d sensor, you need to change environment variables:

```bash
export TURTLEBOT_3D_SENSOR=r200
export TURTLEBOT_STACKS=minimal
```

To have quick shortcuts, consider pasting this code snippet in your `~/.bash_aliases` file:

```bash
alias setr200='export TURTLEBOT_3D_SENSOR=r200 && export TURTLEBOT_STACKS=minimal'
alias setkinect='export TURTLEBOT_3D_SENSOR=kinect && export TURTLEBOT_STACKS=hexagons'
```

## B - Mapping

The only difference from the kinect version of the navigation stack is that you need to start the camera driver before the navigation: `roslaunch realsense_camera realsense_r200_navigation.launch`.

So, the normal flow would be :

```bash
roslaunch turtlebot_bringup minimal.launch
roslaunch realsense_camera realsense_r200_navigation.launch
roslaunch realsense_navigation gmapping.launch
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

You may also want to teleoperate the robot, for instance with the keyboard.

![](doc/img/mapping_screen.png)

Once you're done mapping, you can save the map with the following command:

```bash
rosrun map_server map_saver -f <map-file>
```

> Don't put an extension to your map name, the .*yaml* and .*pgm* files will be created accordingly in you `~/.ros` folder. If you want, you can also provide an absolute path.


## C - Navigation

Once you have a map, you can start the navigation with the following commands

```bash
roslaunch turtlebot_bringup minimal.launch
roslaunch realsense_camera realsense_r200_navigation.launch
roslaunch realsense_navigation navigation_demo.launch map:=<map-file>
```

## D - Simulating a mapping
Now that you know how to map a room using the **R200**, you might need to repeat the process several times, to adjust all the parameters of the algorithm, or to test with depth enhancement. To avoid using the robot, we can simulate the scanning process (= robot moves + camera) thanks to **rosbag**.

### 1. Recording a scan
The first thing to do is to record the scanning process. We will store in a *bag file* the required contents so we can later launch gmapping **on the same input data**.

So we first need to start the turtlebot (+ the teleop of your choice) and the camera:

```bash
roslaunch turtlebot_bringup minimal.launch
roslaunch realsense_camera realsense_r200_navigation.launch
```

Then, we will store the relevent topics in a bag file:

* _/camera/depth/image_raw_
* _/camera/depth/camera_info_
* _/scan_
* _/tf_

![](doc/img/bag_screen.png)
*You can visualize the bagfile with __rqt_bag__*

> Note that we don't need to actually build the map, so calling gmapping is not necessary. But you can do it, to have an idea if you're not moving the robot too fast, for instance.

```bash
rosbag record -O <output.bag> /camera/depth/image_raw /camera/depth/camera_info /mobile_base/commands/velocity /tf
```

You can now begin to move/teleop your robot around the room. Once you're done, kill first the rosbag record process, then the other nodes.

### 2. Playing it back
From now on, you'll be able to simulate this mapping again. To do so, you should simply type this line in a terminal (without any other running node):

```bash
roslaunch realsense_navigation simulate_mapping.launch bag_file:=<output.bag>
```

> The path to the bagfile is relative to the user `~` folder. To change this, use the *home* argument as follows: 
>
> ```bash
> roslaunch realsense simulate_mapping.launch home:=/tmp bag_file:=<output.bag>
> ```

When the RVIZ window is up, you can press `SPACE` in your terminal to start the playback.

Consider taking a look at the launchfile to see the different arguments.
