cmake_minimum_required(VERSION 3.6)
project(src)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        TrackingRenderer/BlackSegmentationRenderer.h
        TrackingRenderer/Colors.h
        TrackingRenderer/OpencvUtils.h
        TrackingRenderer/PersonDataStorage.h
        TrackingRenderer/SegmentationRenderer.h
        TrackingRenderer/TrackingRenderer.cpp
        TrackingRenderer/TrackingRenderer.h
        TrackingRenderer/Viewer.cpp
        TrackingRenderer/Viewer.h
        TrackingRenderer/WinStyleSegmentationRenderer.h
        PersonTrackingSample.cpp
        PersonTrackingSample.h
        PersonTrackingSampleRunner.cpp
        SafeMutex.h)

add_executable(src ${SOURCE_FILES})