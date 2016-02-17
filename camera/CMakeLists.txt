cmake_minimum_required(VERSION 2.8.3)

project(realsense_camera)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  cv_bridge
  image_transport
  camera_info_manager
  tf
  message_generation
  std_msgs
  sensor_msgs
  rostest
  pcl_ros
)

add_service_files(
  FILES
  cameraConfiguration.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(realsense_camera_nodelet src/realsense_camera_nodelet.cpp)
target_link_libraries(realsense_camera_nodelet 
  ${catkin_LIBRARIES}
  /usr/local/lib/librealsense.so
)

add_executable(realsense_camera_test test/realsense_camera_test_node.cpp)
target_link_libraries(realsense_camera_test 
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)
