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
set(CMAKE_SHARED_LINKER_FLAGS "-z noexecstack -z relro -z now ${CMAKE_SHARED_LINKER_FLAGS}")

option(USE_SYSTEM_LIBREALSENSE "Build realsense_camera against system librealsense, not the one built with catkin" OFF)

set(REALSENSE_CATKIN_BASED_DEPS
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
  pcl_ros
)

if(USE_SYSTEM_LIBREALSENSE)
  find_package(realsense REQUIRED)
else()
  list(APPEND REALSENSE_CATKIN_BASED_DEPS librealsense)
endif()

find_package(catkin REQUIRED COMPONENTS
  ${REALSENSE_CATKIN_BASED_DEPS}
)

add_message_files(
  FILES
  IMUInfo.msg
)

add_service_files(
  FILES
  CameraConfiguration.srv
  ForcePower.srv
  SetPower.srv
  IsPowered.srv
  GetIMUInfo.srv
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
  cfg/zr300_params.cfg
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
if(USE_SYSTEM_LIBREALSENSE)
  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS std_msgs message_runtime sensor_msgs
    DEPENDS realsense
    LIBRARIES ${PROJECT_NAME}_nodelet
  )
else()
  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS std_msgs message_runtime sensor_msgs librealsense
    LIBRARIES ${PROJECT_NAME}_nodelet
  )
endif()

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_nodelet src/base_nodelet.cpp src/sync_nodelet.cpp src/r200_nodelet.cpp src/f200_nodelet.cpp src/sr300_nodelet.cpp
  src/zr300_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
)
if(USE_SYSTEM_LIBREALSENSE)
  target_link_libraries(${PROJECT_NAME}_nodelet
    ${realsense_LIBRARY}
    )
endif()

add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME}_nodelet ${catkin_EXPORTED_TARGETS})

add_executable(get_debug_info src/get_debug_info.cpp)
target_link_libraries(get_debug_info ${catkin_LIBRARIES})
if(USE_SYSTEM_LIBREALSENSE)
  target_link_libraries(get_debug_info PRIVATE realsense::realsense)
endif()

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslint REQUIRED)

  # ROS Lint the code
  roslint_cpp()
  roslint_python()
  roslint_add_test()

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
endif()

# Install nodelet library
install(TARGETS ${PROJECT_NAME}_nodelet get_debug_info
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
