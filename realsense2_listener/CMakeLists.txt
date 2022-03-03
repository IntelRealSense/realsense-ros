cmake_minimum_required(VERSION 3.5)
project(realsense2_listener)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
option(BUILD_WITH_OPENMP "Use OpenMP" OFF)
option(SET_USER_BREAK_AT_STARTUP "Set user wait point in startup (for debug)" OFF)

# Compiler Defense Flags
if(UNIX OR APPLE)
  # Linker flags.
  if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    # GCC specific flags. ICC is compatible with them.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z noexecstack -z relro -z now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z noexecstack -z relro -z now")
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # In Clang, -z flags are not compatible, they need to be passed to linker via -Wl.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
  endif()

  # Compiler flags.
  if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    # GCC specific flags.
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 4.9)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector-strong")
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector")
    endif()
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # Clang is compatbile with some of the flags.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector")
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    # Same as above, with exception that ICC compilation crashes with -fPIE option, even
    # though it uses -pie linker option that require -fPIE during compilation. Checksec
    # shows that it generates correct PIE anyway if only -pie is provided.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector")
  endif()

  # Generic flags.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fno-operator-names -Wformat -Wformat-security -Wall")
  # Dot not forward c++ flag to GPU beucause it is not supported
  set( CUDA_PROPAGATE_HOST_FLAGS OFF )
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -D_FORTIFY_SOURCE=2")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

string(TOUPPER "${CMAKE_BUILD_TYPE}" uppercase_CMAKE_BUILD_TYPE)
if (${uppercase_CMAKE_BUILD_TYPE} STREQUAL "RELEASE")
    message(STATUS "Create Release Build.")
    set(CMAKE_CXX_FLAGS "-O2 ${CMAKE_CXX_FLAGS}")
else()
    message(STATUS "Create Debug Build.")
endif()

if(BUILD_WITH_OPENMP)
    find_package(OpenMP)
    if(NOT OpenMP_FOUND)
        message(FATAL_ERROR "\n\n OpenMP is missing!\n\n")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    endif()
endif()

if(SET_USER_BREAK_AT_STARTUP)
	message("GOT FLAG IN CmakeLists.txt")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBPDEBUG")
endif()


find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)


include_directories(include)

set(node_plugins "")

set(SOURCES
    src/realsense_zero_copy_listener.cpp
  )

if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS_DISTRO is not defined." )
endif()
if("$ENV{ROS_DISTRO}" STREQUAL "dashing")
  message(STATUS "Build for ROS2 Dashing")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDASHING")
elseif("$ENV{ROS_DISTRO}" STREQUAL "eloquent")
  message(STATUS "Build for ROS2 eloquent")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DELOQUENT")
elseif("$ENV{ROS_DISTRO}" STREQUAL "foxy")
  message(STATUS "Build for ROS2 Foxy")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DFOXY")
elseif("$ENV{ROS_DISTRO}" STREQUAL "galactic")
  message(STATUS "Build for ROS2 Galactic")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGALACTIC")
elseif("$ENV{ROS_DISTRO}" STREQUAL "rolling")
  message(STATUS "Build for ROS2 Rolling")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DROLLING")
else()
  message(FATAL_ERROR "Unsupported ROS Distribution: " "$ENV{ROS_DISTRO}")
endif()


add_library(${PROJECT_NAME} SHARED
    src/constants.h
    
    ${SOURCES}
)

#target_link_libraries(${PROJECT_NAME}
#    ${realsense2_LIBRARY}
#    )

set(dependencies
  cv_bridge
  image_transport
  rclcpp
  rclcpp_components
  realsense2_camera_msgs
  std_msgs
  std_srvs
  sensor_msgs
  #nav_msgs
  #tf2
  #realsense2
  #tf2_ros
  #diagnostic_updater
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "realsense2_listener::ImageViewNode"
  EXECUTABLE ImageViewNode
)


# Install binaries
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
#install(DIRECTORY 
#    launch
#    DESTINATION share/${PROJECT_NAME}
#    )


# Ament exports
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
