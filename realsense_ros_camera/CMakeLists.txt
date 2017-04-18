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
  tf
)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
 set(OpenMP_LIBS gomp)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(OpenCV 3 REQUIRED)
#find_package(librealsense 1 REQUIRED) librealsense package currently lacks required pkgconfig file

set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -O2 -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")

add_message_files(
  FILES
  IMUInfo.msg
  Extrinsics.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
  LIBRARIES realsense_ros_camera
  CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs
  nodelet
  cv_bridge
  image_transport
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
add_library(realsense_ros_camera
   src/camera_node.cpp
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(realsense_ros_camera
   realsense
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${CMAKE_THREAD_LIBS_INIT}
   ${OpenMP_LIBS}
 )

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_executable(realsense_ros_tests_camera test/camera_core.cpp)
  target_link_libraries(realsense_ros_tests_camera
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    -lpthread -lm
    )
  add_dependencies(realsense_ros_tests_camera ${PROJECT_NAME}_generate_messages_cpp)
  add_dependencies(realsense_ros_tests_camera ${catkin_EXPORTED_TARGETS})
endif()


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES nodelet_plugins.xml 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

