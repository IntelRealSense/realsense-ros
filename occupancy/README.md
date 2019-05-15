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

### Expected output (RViz)
![occupancy_6](https://user-images.githubusercontent.com/28366639/57812351-a5280400-7721-11e9-908e-ac2639b0c8d6.gif)

## Appendix
### Mechanical mount T265 + D435
![mount_Screenshot from 2019-05-15 15-00-17](https://user-images.githubusercontent.com/28366639/57812608-66467e00-7722-11e9-903e-19ecd0a9f2b6.png)
https://github.com/IntelRealSense/realsense-ros/blob/occupancy-mapping/realsense2_camera/meshes/mount_t265_d435.stl

Corresponding extrinsics: https://github.com/IntelRealSense/realsense-ros/blob/occupancy-mapping/realsense2_camera/urdf/mount_t265_d435.urdf.xacro
