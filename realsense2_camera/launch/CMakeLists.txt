cmake_minimum_required(VERSION 2.8.3)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
    roslaunch
  )
  set(LAUNCH_FILES
    demo_pointcloud.launch
    rs_aligned_depth.launch
    rs_camera.launch
    rs_multiple_devices.launch
    rs_rgbd.launch
    view_r410_model.launch
    view_r430_model.launch
  )
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE} USE_TEST_DEPENDENCIES)
  endforeach()
endif()
