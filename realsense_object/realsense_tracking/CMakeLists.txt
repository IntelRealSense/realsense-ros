cmake_minimum_required(VERSION 2.8.3)
project(realsense_tracking)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

set(UP	 "${CMAKE_CURRENT_SOURCE_DIR}/../../../")
set(DEPS "${UP}dependencies")


set(OPENCV_LIBS
	opencv_core
	opencv_imgproc
	opencv_highgui
	opencv_imgcodecs
		)

set(OPENCV_INCLUDE_DIRS ${DEPS}/opencv_3.1/include)

#find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  realsense_or_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs realsense_or_msgs
)

##########
## Build ##
###########

set(BUILD_NODELETE 1)

include_directories(${catkin_INCLUDE_DIRS} ../utils/
	${DEPS}/

)


if(BUILD_NODELETE)
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -DRUN_AS_NODELET" )
add_library(realsense_tracking_nodelet
  				src/realsense_tracking_nodelet.cpp ../utils/utils.cpp)
else(BUILD_NODELETE)
add_executable(	realsense_tracking_nodelet
  				src/realsense_tracking_nodelet.cpp ../utils/utils.cpp)

endif(BUILD_NODELETE)


target_link_libraries(realsense_tracking_nodelet 
	realsense
	realsense_image
	realsense_projection
	realsense_object_recognition

        ${OPENCV_LIBS}
        ${catkin_LIBRARIES}
	${roscpp_LIBRARIES}
	)

add_dependencies(realsense_tracking_nodelet realsense_or_msgs_gencpp)
