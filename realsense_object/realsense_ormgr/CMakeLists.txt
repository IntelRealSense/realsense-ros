cmake_minimum_required(VERSION 2.8.3)
project(realsense_ormgr)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

set(UP	 "${CMAKE_CURRENT_SOURCE_DIR}/../../../")
set(DEPS "${UP}dependencies")
set(OPENCV_LIBS
	${DEPS}/opencv_3.1/lib/libopencv_core.so
	${DEPS}/opencv_3.1/lib/libopencv_imgproc.so
	${DEPS}/opencv_3.1/lib/libopencv_highgui.so
	${DEPS}/opencv_3.1/lib/libopencv_imgcodecs.so
		)

set(OPENCV_INCLUDE_DIRS ${DEPS}/opencv_3.1/include)

#find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  realsense_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs realsense_msgs
)

##########
## Build ##
###########

set(BUILD_NODELETE 1)

include_directories(${catkin_INCLUDE_DIRS}
	${DEPS}/
	${DEPS}/realsense_sdk/include/
	${DEPS}/librealsense/include/
	${DEPS}/realsense_object_recognition/include/
	../utils/
	include
	${OPENCV_INCLUDE_DIRS} 
)

link_directories(${DEPS}/opencv_3.1/lib)

if(BUILD_NODELETE)
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -DRUN_AS_NODELET" )
add_library(realsense_ormgr_nodelet
  				src/realsense_ormgr_nodelet.cpp)
else(BUILD_NODELETE)

add_executable(	realsense_ormgr_nodelet
  				src/realsense_ormgr_nodelet.cpp)

endif(BUILD_NODELETE)

## Specify libraries to link a library or executable target against

target_link_libraries(realsense_ormgr_nodelet 
						${catkin_LIBRARIES} 
						${roscpp_LIBRARIES} 
)


target_link_libraries(realsense_ormgr_nodelet)


#add_dependencies(realsense_recognition_nodelet realsense_msgs_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(realsense_ormgr_nodelet realsense_msgs_generate_messages_cpp)

