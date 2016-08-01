cmake_minimum_required(VERSION 2.8.3)

project(realsense_camera)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# View the makefile commands during build
#set(CMAKE_VERBOSE_MAKEFILE on)

# Set compile flags
set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -O2 -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")
# Flags executables
set(CMAKE_EXE_LINKER_FLAGS "-pie -z noexecstack -z relro -z now")
# Flags shared libraries
set(CMAKE_SHARED_LINKER_FLAGS "-z noexecstack -z relro -z now")

find_library(REALSENSE_LIB realsense)

# This is optional until the first release of librealsense as a ROS package
find_package(catkin OPTIONAL_COMPONENTS librealsense)
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  nodelet
  cv_bridge
  image_transport
  camera_info_manager
  tf
  message_generation
  std_msgs
  sensor_msgs
  rostest
  pcl_ros
)

add_service_files(
  FILES
  cameraConfiguration.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/r200_params.cfg
  cfg/f200_params.cfg
  cfg/sr300_params.cfg
)

#################################
# catkin specific configuration #
#################################
# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if you package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# CATKIN_DEPENDS: catkin_packages dependent projects also need
# DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS librealsense std_msgs message_runtime sensor_msgs
  LIBRARIES ${PROJECT_NAME}_nodelet
)

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_nodelet src/base_nodelet.cpp src/r200_nodelet.cpp src/f200_nodelet.cpp src/sr300_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
  # This should be removed once librealsense is a required component
  ${REALSENSE_LIB}
)
add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME}_nodelet ${catkin_EXPORTED_TARGETS})

add_executable(tests_camera_core test/camera_core.cpp)
target_link_libraries(tests_camera_core
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)
add_dependencies(tests_camera_core ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)
add_dependencies(tests_camera_core ${catkin_EXPORTED_TARGETS})

add_executable(tests_rgbd_topics test/rgbd_topics.cpp)
target_link_libraries(tests_rgbd_topics
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)
add_dependencies(tests_rgbd_topics ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(tests_rgbd_topics ${catkin_EXPORTED_TARGETS})

# Install nodelet library
install(TARGETS ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install rviz files
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

# Install xml files
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
