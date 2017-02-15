cmake_minimum_required(VERSION 2.8.3)
project(realsense_ros_camera)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  nodelet
  cv_bridge
  image_transport
)

find_package(OpenCV 3 REQUIRED)

set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -O2 -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")
set(CMAKE_THREAD_LIBS_INIT pthread)

 add_message_files(
   FILES
  IMUInfo.msg
  Extrinsics.msg
 )

 generate_messages(
   DEPENDENCIES
   sensor_msgs   std_msgs
 )

catkin_package(
  LIBRARIES realsense_ros_camera
  CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs
  nodelet
  cv_bridge
  image_transport
)

include_directories(
/usr/local/include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
add_library(realsense_ros_camera
   src/camera_node.cpp
)

add_dependencies(realsense_ros_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(realsense_ros_camera
   realsense
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${CMAKE_THREAD_LIBS_INIT}
 )

# Install nodelet library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install xml files
install(FILES test_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

