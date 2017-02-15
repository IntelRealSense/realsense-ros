cmake_minimum_required(VERSION 2.8.3)
project(realsense_ros_slam)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  image_transport
  message_filters
  message_generation
  realsense_ros_camera
  tf2
  )
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)
set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -O2 -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")

set(ROOT      "${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_THREAD_LIBS_INIT pthread)

generate_messages(
  DEPENDENCIES
  sensor_msgs 
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES realsense_ros_slam
  CATKIN_DEPENDS nodelet 
                 roscpp 
                 rospy 
                 sensor_msgs 
                 std_msgs 
                 geometry_msgs
                 message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ROOT}/include
  ${EIGEN3_INCLUDE_DIR}
)
file(GLOB SOURCES
  src/slam_nodelet.cpp)

add_library(${PROJECT_NAME} ${SOURCES})

set(LIBRARIES_TO_LINK
    realsense
    realsense_image
    realsense_slam
    SP_Core
    tracker
    ${CMAKE_THREAD_LIBS_INIT}
    ${OpenCV_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${LIBRARIES_TO_LINK}
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

# Install lib files
install(DIRECTORY lib/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/lib
)
# Install include files
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/include
)

# Install xml files
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

