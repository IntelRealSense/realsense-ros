cmake_minimum_required(VERSION 3.3)
project(src)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
    Core/LinuxProjection.cpp
    Core/LinuxProjection.h
    Core/LinuxPxcDepthImage.cpp
    Core/LinuxPxcDepthImage.h
    Core/LinuxPxcImage.cpp
    Core/LinuxPxcImage.h
    PersonTrackingInitiator.cpp
    PersonTrackingInitiator.h
    PersonTrackingNodelet.cpp
    PersonTrackingNodelet.h
    PersonTrackingRunner.cpp)

add_executable(src ${SOURCE_FILES})