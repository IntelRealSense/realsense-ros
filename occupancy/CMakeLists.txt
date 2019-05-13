cmake_minimum_required(VERSION 3.1)
project(occupancy)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    tf
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
    src/MathUtil.cpp
    src/SP_CommonVisionUtil.cpp
    src/SP_MapManager.cpp
    src/occupancy_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)
