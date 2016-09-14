cmake_minimum_required(VERSION 2.8.3)
project(realsense_person_tracking)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPT_MW_DEV")


##############
# Variables ##
##############
set(PERSON_TRACKING_LIB "librealsense_persontracking.so")

########
# ROS ##
########


find_package(catkin REQUIRED COMPONENTS
  roscpp  
  rospy
  image_transport
  std_msgs
  message_generation
  sensor_msgs
  geometry_msgs
  realsense_msgs
  realsense_srvs
)

add_message_files(
  FILES
  PersonTrackingOut.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  realsense_msgs
  realsense_srvs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs geometry_msgs realsense_msgs realsense_srvs
)


###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/librealsense/pt/
)

add_executable(realsense_person_tracking
  src/PersonTrackingRunner.cpp
  src/PersonTrackingNodelet.cpp
  src/PersonTrackingPublisher.cpp
  src/PersonTrackingServer.cpp
  src/RosLogWriter.cpp
)

target_link_libraries(realsense_person_tracking
  ${catkin_LIBRARIES} 
  ${PERSON_TRACKING_LIB}
  rs_projection
  /usr/lib/x86_64-linux-gnu/libpng.a
  /usr/lib/x86_64-linux-gnu/libjpeg.a

  /usr/lib/x86_64-linux-gnu/libgtk-x11-2.0.so
  /usr/lib/x86_64-linux-gnu/libglib-2.0.so
  /usr/lib/x86_64-linux-gnu/libgthread-2.0.so
)



