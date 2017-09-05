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

find_package(OpenCV REQUIRED)

if (CMAKE_BUILD_TYPE EQUAL "RELEASE")
    set(CMAKE_CXX_FLAGS "-O2 ${CMAKE_CXX_FLAGS}")
endif()

set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -fopenmp -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")

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

# DS5 Node
catkin_package(
    LIBRARIES ${PROJECT_NAME}_ds5
    CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs librealsense2
    nodelet
    cv_bridge
    image_transport
    )

add_library(${PROJECT_NAME}_ds5
    include/constants.h
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

install(TARGETS ${PROJECT_NAME}_ds5
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
