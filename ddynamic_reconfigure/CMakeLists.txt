cmake_minimum_required(VERSION 2.8.3)
project(ddynamic_reconfigure)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure roscpp message_generation std_msgs)

set(CMAKE_CXX_STANDARD 98)

##############
## Services ##
##############

add_service_files(DIRECTORY test FILES TutorialParams.srv)
generate_messages(DEPENDENCIES std_msgs)

############
## Catkin ##
############

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS message_runtime std_msgs)

#############
## Library ##
#############

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME}
        src/ddynamic_reconfigure.cpp
        include/ddynamic_reconfigure/dd_param.h
        src/param/dd_int_param.cpp
        include/ddynamic_reconfigure/dd_value.h
        src/param/dd_double_param.cpp
        src/param/dd_bool_param.cpp
        src/param/dd_string_param.cpp
        src/param/dd_enum_param.cpp
        include/ddynamic_reconfigure/param/dd_all_params.h src/dd_param.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    ## DDynamic tester
    add_rostest_gtest(ddynamic_reconfigure-test test/ddynamic_reconfigure.test test/test_ddynamic_reconfigure.cpp)
    target_link_libraries(ddynamic_reconfigure-test ${PROJECT_NAME})

    ## DDParam tester
    foreach (param_type int double bool string enum)
        add_rostest_gtest(dd_${param_type}-test test/dd_param/dd_${param_type}.test test/dd_param/test_dd_${param_type}.cpp)
        target_link_libraries(dd_${param_type}-test ${PROJECT_NAME})
    endforeach ()

    ## DDValue tester
    add_rostest_gtest(dd_value-test test/dd_value.test test/test_dd_value.cpp)
    target_link_libraries(dd_value-test ${PROJECT_NAME})

    ## Full scale tester
    add_executable(dd_server test/dd_full_scale/dd_server.cpp)
    target_link_libraries(dd_server ${PROJECT_NAME} ${catkin_LIBRARIES})
    add_dependencies(dd_server ${PROJECT_NAME}_gencpp)

    add_rostest_gtest(dd_full_scale-test test/dd_full_scale/dd_full_scale.test test/dd_full_scale/dd_client.cpp)
    target_link_libraries(dd_full_scale-test ${PROJECT_NAME})
endif(CATKIN_ENABLE_TESTING)
