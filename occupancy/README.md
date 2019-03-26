This ROS package can be used to generate a 2D occupancy map based on depth images, for example from Intel(R) Realsense(TM) D435 (or D415), and poses, for example from Intel(R) Realsense(TM) T265.
Internally, it uses a 3D representation to transform point clouds into a common reference frame, using T265 poses, and to accumulate information over time.
The accumulated measurements are mapped to a probability of a field of beeing free (0) to occupied (100).
This output can be used for robot navigation.

### How to
create catkin workspace:
```
source /opt/ros/kinetic/setup.bash`
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin build
```

checkout package:
```
cd src
git clone <URL_TO_REPOSITORY>
```

build:
```
cd ..
catkin build occupancy
```

run:
```
source devel/setup.bash
roslaunch occupancy occupancy_live_rviz.launch
```
