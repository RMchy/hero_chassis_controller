cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Treat warnings as errors. GOOD for the code-style requirement (assignment 3),
## but enable -Werror only AFTER the code compiles cleanly, otherwise every
## tiny warning blocks the build while you are learning.
add_definitions(-Wall)
# add_definitions(-Werror)   # TODO: uncomment once build is warning-free

## Find catkin macros and libraries
find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    roslint
    controller_interface
    hardware_interface
    forward_command_controller
    pluginlib
    control_toolbox
    std_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    tf2_geometry_msgs
    dynamic_reconfigure
    )

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS
    include
    LIBRARIES
    ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    roslint
    controller_interface
    hardware_interface
    forward_command_controller
    pluginlib
    control_toolbox
    geometry_msgs
    nav_msgs
    tf2_ros
    tf2_geometry_msgs
    dynamic_reconfigure
    DEPENDS
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

## A ros_control controller is a SHARED LIBRARY (.so), NOT an executable.
## pluginlib loads it into gzserver at runtime.
add_library(${PROJECT_NAME}
    src/${PROJECT_NAME}.cpp
    )

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )

#############
## Install ##
#############

install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

##########################
## Static code analysis ##
##########################

## Style check for assignment 3 (ROS C++ / Google style). Clean these warnings.
roslint_cpp()
