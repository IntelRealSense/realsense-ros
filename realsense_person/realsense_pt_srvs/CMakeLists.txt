cmake_minimum_required(VERSION 2.8.3)
project(realsense_pt_srvs)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)


add_service_files(
  FILES
  TrackingConfig.srv
  TrackingRequest.srv
  RecognitionRequest.srv
  LoadRecognitionDB.srv
  SaveRecognitionDB.srv
  RecognitionImgRequest.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs 
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
