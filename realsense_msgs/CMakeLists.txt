cmake_minimum_required(VERSION 2.8.3)
project(realsense_msgs)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  sensor_msgs
  geometry_msgs
)

find_package(PCL 1.7 REQUIRED COMPONENTS common io)

add_message_files(
	FILES
# PT
	Landmark.msg
	User.msg
	UserInfo.msg
	Frame.msg
        Gestures.msg
        Pointing.msg

# Object
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
  sensor_msgs
  geometry_msgs
  
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs geometry_msgs 
)

include_directories(
  ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} /usr/include /usr/include/pcl-1.7 /usr/include/eigen3/
)

include_directories()
###########
## Build ##
###########

