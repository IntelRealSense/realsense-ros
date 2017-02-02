cmake_minimum_required(VERSION 2.8.3)
project(realsense_orview)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  realsense_or_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs realsense_or_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS} /usr/include
)


##########
## Build ##
###########

set(BUILD_NODELETE 1)


if(BUILD_NODELETE)
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -DRUN_AS_NODELET" )
add_library(realsense_orview_nodelet
  				src/realsense_orview_nodelet.cpp
	src/realsense_orview_nodelet.cpp ../utils/GUI_utils.cpp)

else(BUILD_NODELETE)
add_executable(	realsense_orview_nodelet
  				src/realsense_orview_nodelet.cpp
		src/realsense_orview_nodelet.cpp ../utils/GUI_utils.cpp)
endif(BUILD_NODELETE)

## Specify libraries to link a library or executable target against

target_link_libraries(realsense_orview_nodelet 
						${catkin_LIBRARIES} 
						${roscpp_LIBRARIES} 
)


target_link_libraries(realsense_orview_nodelet)


#add_dependencies(realsense_recognition_nodelet realsense_or_msgs_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(realsense_orview_nodelet realsense_or_msgs_gencpp)
