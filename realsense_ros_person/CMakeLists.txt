cmake_minimum_required(VERSION 2.8.3)
project(realsense_ros_person)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    sensor_msgs
    nodelet
    message_filters
    message_generation
    roscpp
    roslib
    rostime
    image_transport
    cv_bridge
    roscpp_serialization
    rospy
  )

include(FindPkgConfig)
pkg_check_modules(PERSONTRACKING REQUIRED realsense_persontracking)

# Set compile flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fexceptions -frtti")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ")

set(PNG_LIBRARIES png)
set(JPEG_LIBRARIES jpeg)
set(CMAKE_THREAD_LIBS_INIT pthread)

set(ROOT                "${CMAKE_CURRENT_SOURCE_DIR}")


add_message_files(
    FILES
    Landmark.msg
    LandmarksInfo.msg
    SkeletonJoint.msg
    User.msg
    UserInfo.msg
    Gesture.msg
    Gestures.msg
    Pointing.msg
    Wave.msg
    Frame.msg
    FrameTest.msg
    RectWithConfidence.msg
    EulerAngles.msg
    EulerAnglesWithConfidence.msg
    PersonModuleState.msg
)

add_service_files(
    FILES
    TrackingConfig.srv
    StartTracking.srv
    StopTracking.srv
    Recognition.srv
    RecognitionRegister.srv
    LoadRecognitionDB.srv
    SaveRecognitionDB.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PERSONTRACKING_INCLUDE_DIRS}
    ${ROOT}/src/apiwrapper/server/
    ${ROOT}/src/apiwrapper/publisher/
    ${ROOT}/src/apiwrapper/helpers/

)

set( PT_LINK_LIBS
    ${PERSONTRACKING_LIBRARIES}
-Wl,--start-group
        ${OpenCV_LIBS}
        ${JPEG_LIBRARIES}
-Wl,--end-group
        ${PNG_LIBRARIES}
        ${GTK2_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
     )

catkin_package(
    LIBRARIES realsense_ros_person
    CATKIN_DEPENDS nodelet roscpp std_msgs image_transport sensor_msgs message_filters roslib rostime cv_bridge roscpp_serialization rospy

)

file(GLOB ROS_WRAPPER_SOURCES
    "src/apiwrapper/*.cpp"
    "src/apiwrapper/helpers/*.cpp"
    "src/apiwrapper/publisher/*.cpp"
    "src/apiwrapper/server/*.cpp"
)

# build person tracking API ROS wrapper
add_library(realsense_ros_person
    ${ROS_WRAPPER_SOURCES}
)

target_link_libraries(realsense_ros_person
    ${PT_LINK_LIBS}
    ${catkin_LIBRARIES}
)


add_dependencies(realsense_ros_person ${PROJECT_NAME}_generate_messages_cpp)


add_executable(${PROJECT_NAME}_sample
    ${ROOT}/src/sample/PersonTrackingSampleRunner.cpp
    ${ROOT}/src/sample/PersonTrackingSample.cpp
    ${ROOT}/src/sample/TrackingRenderer/TrackingRenderer.cpp
    ${ROOT}/src/sample/TrackingRenderer/Viewer.cpp
)

target_link_libraries(${PROJECT_NAME}_sample
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )


add_dependencies(${PROJECT_NAME}_sample ${PROJECT_NAME}_generate_messages_cpp)

if(REALSENSE_ENABLE_TESTING)
  # build person unit tests
  find_package(rostest REQUIRED)

  catkin_download_test_data(
    person_detection.bag
    https://s3-eu-west-1.amazonaws.com/realsense-rostest-public/realsense_ros_person/person_detection.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    MD5 8a4a9db6b57d142ba8c408fda65b72c2)

  catkin_download_test_data(
    recognition.bag
    https://s3-eu-west-1.amazonaws.com/realsense-rostest-public/realsense_ros_person/recognition.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    MD5 f62e471583946bdc1314a250bde7d073)

  catkin_download_test_data(
    wave_detection.bag
    https://s3-eu-west-1.amazonaws.com/realsense-rostest-public/realsense_ros_person/wave_detection.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    MD5 716657342c4dd72cfd7abbfdc8d16b11)

  add_executable(tests_person test/person_test.cpp)
  add_rostest(test/person_detection.test)
  add_rostest(test/person_tracking.test)
  add_rostest(test/recognition.test)
  add_rostest(test/wave_detection.test)
  target_link_libraries(tests_person
        ${catkin_LIBRARIES}
        ${GTEST_LIBRARIES}
        )
  add_dependencies(tests_person realsense_ros_person)
  add_dependencies(tests_person person_detection.bag recognition.bag wave_detection.bag) 

endif()

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install xml files
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

