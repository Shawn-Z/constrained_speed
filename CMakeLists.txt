cmake_minimum_required(VERSION 2.8.3)
project(constrained_speed)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=gnu++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(catkin_deps
        roslib
        nodelet
        roscpp
        nav_msgs
        std_msgs
        plan2control_msgs
        anm_msgs
        speed_debug_msgs
        sensor_msgs
        control_msgs
        sensor_driver_msgs
        rosbag)

find_package(catkin REQUIRED
        COMPONENTS
        ${catkin_deps})

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS ${catkin_deps}
        LIBRARIES ${PROJECT_NAME}_nodelet
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(constrained_speed_nodelet
        src/ConstrainedSpeedNodelet.cpp)
add_library(constrained_speed_analysis
        src/ConstrainedSpeedAnalysis.cpp)

add_executable(constrained_speed_node
        src/ConstrainedSpeedNode.cpp)

target_link_libraries(
        constrained_speed_nodelet
        ${catkin_LIBRARIES}
)

target_link_libraries(
        constrained_speed_analysis
        ${catkin_LIBRARIES}
)

target_link_libraries(
        constrained_speed_node
        ${catkin_LIBRARIES}
)