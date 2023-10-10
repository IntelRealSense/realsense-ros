# Align Depth to Color
This example shows how to start the camera node and align depth stream to color stream.
```
ros2 launch realsense2_camera rs_align_depth_launch.py
```

The aligned image will be published to the topic "/aligned_depth_to_color/image_raw"

Also, align depth to color can enabled by following cmd:
```
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```
