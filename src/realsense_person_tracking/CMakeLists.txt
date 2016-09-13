cmake_minimum_required(VERSION 2.8.3)
project(realsense_person_tracking)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPT_MW_DEV")


##############
# Variables ##
##############


set(LIBS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../libs/persontracking")

set(PERSON_TRACKING_LIB "${LIBS_DIR}/lib/libpxcpersontracking.so")

set(IPP_LIBS "${LIBS_DIR}/lib/ipp")


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
  ${LIBS_DIR}/include/adapter
  ${LIBS_DIR}/include/pxcinclude
  ${LIBS_DIR}/include
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
  ${IPP_LIBS}/libipprs.a
  ${IPP_LIBS}/libippi.a
  ${IPP_LIBS}/libipps.a
  ${IPP_LIBS}/libippdc.a
  ${IPP_LIBS}/libippcv.a
  ${IPP_LIBS}/libippcc.a
  ${IPP_LIBS}/libippch.a
  ${IPP_LIBS}/libippvm.a
  ${IPP_LIBS}/libippcore.a

  /usr/lib/x86_64-linux-gnu/libpng.a
  /usr/lib/x86_64-linux-gnu/libjpeg.a

  /usr/lib/x86_64-linux-gnu/libgtk-x11-2.0.so
  /usr/lib/x86_64-linux-gnu/libglib-2.0.so
  /usr/lib/x86_64-linux-gnu/libgthread-2.0.so
)



