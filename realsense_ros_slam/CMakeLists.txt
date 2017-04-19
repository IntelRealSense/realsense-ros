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
  tf
  tf2
  )
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)
set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -O2 -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")

set(ROOT      "${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_THREAD_LIBS_INIT pthread)

add_message_files(
  FILES
  TrackingAccuracy.msg
)

add_service_files(
  FILES
  Reset.srv
  SaveOutput.srv
)

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
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${LIBRARIES_TO_LINK}
)

if (REALSENSE_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_download_test_data(
    slam_room_1.bag
    https://s3-eu-west-1.amazonaws.com/realsense-rostest-public/realsense_ros_slam/slam_room_1.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    MD5 73200cdfbde1d83851e23f58d9fbd9ab)

  add_executable(tests_slam test/slam_test.cpp)
  target_link_libraries(tests_slam
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    )
  add_dependencies(tests_slam ${PROJECT_NAME}_generate_messages_cpp)
  add_dependencies(tests_slam ${catkin_EXPORTED_TARGETS})
  add_dependencies(tests_slam slam_room_1.bag)
endif()

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
