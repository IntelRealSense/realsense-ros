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

find_package(OpenCV 3 REQUIRED)

#find_package(PkgConfig)
#pkg_search_module(librealsense REQUIRED) librealsense package currently lacks required pkgconfig file
#pkg_search_module(realsense2 REQUIRED realsense2)

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

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# DS4 Node
catkin_package(
  LIBRARIES realsense_ros_camera
  CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs
  nodelet
  cv_bridge
  image_transport
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
 )


# DS5 Node
catkin_package(
  LIBRARIES ${PROJECT_NAME}_ds5
  CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs
  nodelet
  cv_bridge
  image_transport
)

add_library(${PROJECT_NAME}_ds5
   src/ds5_camera_node.cpp
)

add_dependencies(${PROJECT_NAME}_ds5 ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_ds5 ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_ds5
   realsense2
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${CMAKE_THREAD_LIBS_INIT}
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

