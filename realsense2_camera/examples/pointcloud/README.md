# PointCloud Visualization
The following example starts the camera and simultaneously opens RViz GUI to visualize the published pointcloud.
```
ros2 launch realsense2_camera rs_pointcloud_launch.py
```

Alternatively, start the camera terminal 1:
```
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```
and in terminal 2 open rviz to visualize pointcloud.

# PointCloud with different coordinate systems
This example opens rviz and shows the camera model with different coordinate systems and the pointcloud, so it presents the pointcloud and the camera together.
```
ros2 launch realsense2_camera rs_d455_pointcloud_launch.py
```