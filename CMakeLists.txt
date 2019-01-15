cmake_minimum_required(VERSION 2.8.3)
project(constrained_speed)

OPTION(TOYOTA “ON-for-toyota-or-OFF-for-else” ON)
OPTION(TANK_6T “ON-for-tank_6t-or-OFF-for-else” OFF)
OPTION(NORTH_COLA “ON-for-tank_6t-or-OFF-for-else” OFF)

add_compile_options(-std=c++11)

set(catkin_deps
        roscpp
        plan2control_msgs
        speed_debug_msgs
        control_msgs
        dymidetect2speed_msgs
        sensor_driver_msgs
        speed_ctrl_msgs
        lanelet_map_msgs
)

find_package(catkin REQUIRED
        COMPONENTS
        rosparam_handler
        dynamic_reconfigure
        ${catkin_deps}
)

if (TOYOTA)
    add_definitions(-D TOYOTA)
    generate_ros_parameter_files(cfg/toyota/default/cs_method1.params)
endif (TOYOTA)

if (TANK_6T)
    add_definitions(-D TANK_6T)
    generate_ros_parameter_files(cfg/6t/default/cs_method1.params)
endif (TANK_6T)

if (NORTH_COLA)
    add_definitions(-D NORTH_COLA)
    generate_ros_parameter_files(cfg/north_cola/default/cs_method1.params)
endif (NORTH_COLA)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS ${catkin_deps}
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(constrained_speed_node
        src/ConstrainedSpeedNode.cpp
        src/CSMethod1.cpp
        src/Points_Attributes.cpp
)

add_dependencies(constrained_speed_node
        ${PROJECT_NAME}_gencfg
)

target_link_libraries(constrained_speed_node
        ${catkin_LIBRARIES}
)
