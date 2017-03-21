cmake_minimum_required(VERSION 2.8.3)
project(realsense_ros_object)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

#set(UP	 "${CMAKE_CURRENT_SOURCE_DIR}/../../../")
#set(DEPS "${UP}dependencies")


find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
#  realsense_ros_camera
) 

add_message_files( FILES
	Object.msg
	ObjectArray.msg
	ObjectInBox.msg
	ObjectsInBoxes.msg
	TrackedObject.msg
	TrackedObjectsArray.msg
	UI.msg
	cpu_gpu.msg
	Rect.msg
	Location3D.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs geometry_msgs 
  LIBRARIES ${PROJECT_NAME}
)

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -DRUN_AS_NODELET" )

# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/realsense_localization_nodelet.cpp src/realsense_recognition_nodelet.cpp src/realsense_tracking_nodelet.cpp src/utils.cpp)

target_link_libraries(${PROJECT_NAME}
	realsense
	realsense_image
	realsense_projection
	realsense_object_recognition
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


# Viewer
add_library(${PROJECT_NAME}_viewer src/viewer/realsense_orview_nodelet.cpp src/viewer/GUI_utils.cpp) 

target_link_libraries(${PROJECT_NAME}_viewer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

if (REALSENSE_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_download_test_data(
    detection.bag
    https://s3-eu-west-1.amazonaws.com/realsense-rostest-public/realsense_ros_object/detection.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    MD5 50afdbdd5d3f761612e1643f2a3281f1)

  add_executable(tests_object test/object_test.cpp)
  target_link_libraries(tests_object
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    -lpthread -lm
    )

  add_dependencies(tests_object ${PROJECT_NAME}_generate_messages_cpp)
  add_dependencies(tests_object ${catkin_EXPORTED_TARGETS})
  add_dependencies(tests_object detection.bag)
endif()

add_dependencies(${PROJECT_NAME}_viewer ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_viewer ${catkin_EXPORTED_TARGETS})


# Sample1: Localization and Tracking
add_library(${PROJECT_NAME}_sample1 src/sample-localize-track/realsense_ormgr_nodelet.cpp)

target_link_libraries(${PROJECT_NAME}_sample1
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_sample1 ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_sample1 ${catkin_EXPORTED_TARGETS})


# Install nodelet library
install(TARGETS ${PROJECT_NAME}
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

# Install xml files
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
