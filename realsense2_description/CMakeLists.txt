# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2022 Intel Corporation. All Rights Reserved.

cmake_minimum_required(VERSION 3.5)
project(realsense2_description)

find_package(ament_cmake REQUIRED)

# Install files
install(DIRECTORY 
        launch 
        meshes 
        rviz 
        urdf
        DESTINATION share/${PROJECT_NAME})

ament_package()
