cmake_minimum_required(VERSION 3.5)
project(realsense2_description)

find_package(ament_cmake REQUIRED)

# Install files
install(DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Test code should be rewritten in ROS2
# Tests
# if (CATKIN_ENABLE_TESTING)
#   catkin_add_nosetests(tests)
# endif()

ament_package()
