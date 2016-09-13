cmake_minimum_required(VERSION 2.8.3)
project(realsense_person_tracking_test)

set(CMAKE_CXX_FLAGS "-g -std=c++0x -DPT_ROS ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  std_msgs
  message_generation
  sensor_msgs
  geometry_msgs
  realsense_srvs
  realsense_person_tracking	
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs geometry_msgs realsense_srvs realsense_person_tracking
)

include_directories(																																													
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

###########
## Build ##
###########

add_executable(realsense_person_tracking_test																																	
  src/PersonTrackingTestRunner.cpp
  src/PersonTrackingTest.cpp
  src/TrackingRenderer/TrackingRenderer.cpp
  src/TrackingRenderer/Viewer.cpp
)																																						

target_link_libraries(realsense_person_tracking_test 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
)

add_dependencies(realsense_person_tracking_test realsense_person_tracking_generate_messages_cpp)



