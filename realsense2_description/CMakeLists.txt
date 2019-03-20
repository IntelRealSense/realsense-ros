cmake_minimum_required(VERSION 2.8.3)
project(realsense2_description)

find_package(catkin REQUIRED COMPONENTS
    )

# RealSense ROS Node
catkin_package(
    )


# Install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )

# Install rviz files
install(DIRECTORY rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
    )

# Install urdf files
install(DIRECTORY urdf/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
    )

# Install mesh files
install(DIRECTORY meshes/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
    )
