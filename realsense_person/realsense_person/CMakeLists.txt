cmake_minimum_required(VERSION 2.8.3)
project(realsense_pt_demo)

find_package(catkin REQUIRED COMPONENTS
  std_msgs 
  sensor_msgs
  nodelet
  message_filters
  roscpp
  roslib
  rostime
  image_transport
  cv_bridge
  roscpp_serialization
  rospy
  realsense_pt_msgs
  realsense_srvs
  )
find_package(OpenCV 3 REQUIRED)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPT_MW_DEV")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -fmessage-length=0 --std=c++11 -pthread -fPIC -std=c++0x -fexceptions -frtti -ffunction-sections -fdata-sections")

set(PNG_LIBRARIES png)
set(JPEG_LIBRARIES jpeg)
set(CMAKE_THREAD_LIBS_INIT pthread)

set(ROOT                "${CMAKE_CURRENT_SOURCE_DIR}")
set(SAMPLES_ROOT    "${CMAKE_CURRENT_SOURCE_DIR}/src")

add_definitions(-DPERSON_TRACKING_DATA_FILES="/usr/share/librealsense/pt/data/")

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/librealsense/pt/
  ${OpenCV_INCLUDE_DIRS}
  ${SAMPLES_ROOT}/Common
  ${SAMPLES_ROOT}/Common/TrackingRenderer
  ${SAMPLES_ROOT}/Common/PersonTracker
  )
set( PROJECT_LINK_LIBS
    realsense
    realsense_persontracking
    realsense_projection
    realsense_image
    realsense_playback
    realsense_record
    realsense_log_utils
    ${JPEG_LIBRARIES}
    ${PNG_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${OpenCV_LIBRARIES}
   )
catkin_package(
 LIBRARIES realsense_pt_demo
 CATKIN_DEPENDS nodelet roscpp std_msgs image_transport sensor_msgs message_filters roslib rostime cv_bridge roscpp_serialization rospy
)

file(GLOB SOURCES
    "src/realsense_pt_demo_nodelet.cpp"
    "src/PersonTrackingPublisher.cpp"
    "src/PersonTrackingServer.cpp"
    "src/Common/TrackingRenderer/*.cpp"
    "src/Common/PersonTracker/*.cpp"
    "src/Common/*.cpp"
)

add_library(realsense_pt_demo
	${SOURCES}
 )

 target_link_libraries(realsense_pt_demo
	 ${PROJECT_LINK_LIBS}
      	 ${catkin_LIBRARIES}
	 )
 add_dependencies(realsense_pt_demo realsense_pt_msgs_gencpp realsense_srvs_gencpp)


# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install xml files
install(FILES test_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

