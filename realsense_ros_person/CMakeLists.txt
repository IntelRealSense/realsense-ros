cmake_minimum_required(VERSION 2.8.3)
project(realsense_ros_person)

find_package(catkin REQUIRED COMPONENTS
  std_msgs 
  sensor_msgs
  nodelet
  message_filters
  geometry_msgs
  message_generation
  roscpp
  roslib
  rostime
  image_transport
  cv_bridge
  roscpp_serialization
  rospy

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

add_message_files(
  FILES
  Landmark.msg
  User.msg
  UserInfo.msg
  Frame.msg
  Gestures.msg
  Pointing.msg
)

add_service_files(
  FILES
  LoadRecognitionDB.srv
  RecognitionImgRequest.srv
  RecognitionRequest.srv
  SaveRecognitionDB.srv
  TrackingConfig.srv
  TrackingRequest.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

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
 LIBRARIES realsense_ros_person
 CATKIN_DEPENDS nodelet roscpp std_msgs image_transport sensor_msgs geometry_msgs message_runtime message_filters roslib rostime cv_bridge roscpp_serialization rospy
)

file(GLOB SOURCES
    "src/realsense_ros_person_nodelet.cpp"
    "src/PersonTrackingPublisher.cpp"
    "src/PersonTrackingServer.cpp"
    "src/Common/TrackingRenderer/*.cpp"
    "src/Common/PersonTracker/*.cpp"
    "src/Common/*.cpp"
)

add_library(realsense_ros_person
	${SOURCES}
 )

 target_link_libraries(realsense_ros_person
	 ${PROJECT_LINK_LIBS}
      	 ${catkin_LIBRARIES}
	 )

 add_dependencies(realsense_ros_person ${PROJECT_NAME}_gencpp)


# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install xml files
install(FILES test_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

