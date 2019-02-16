cmake_minimum_required(VERSION 2.8.3)
project(constrained_speed)

add_compile_options(-std=c++11)

set(catkin_deps
        roscpp
        plan2control_msgs
        speed_debug_msgs
        control_msgs
        #dymidetect2speed_msgs
        #sensor_driver_msgs
        speed_ctrl_msgs
        lanelet_map_msgs
        three_one_msgs
)

find_package(catkin REQUIRED
        COMPONENTS
        rosparam_handler
        dynamic_reconfigure
        ${catkin_deps}
)

generate_ros_parameter_files(cfg/cs_method2.params)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS ${catkin_deps}
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)

include_directories(
        include
        include/shawn
        ${catkin_INCLUDE_DIRS}
)

add_executable(constrained_speed
        src/ConstrainedSpeedNode.cpp
        src/CSMethod2.cpp
        include/ThreeOnePublish.hpp include/DEFINEs.hpp)

add_dependencies(constrained_speed
        ${PROJECT_NAME}_gencfg
)

target_link_libraries(constrained_speed
        ${catkin_LIBRARIES}
        glog
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(tests_constrained_speed test/constrained_speed.test test/test.cpp)
    target_link_libraries(tests_constrained_speed ${catkin_LIBRARIES})
endif()