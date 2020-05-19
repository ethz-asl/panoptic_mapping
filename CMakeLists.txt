cmake_minimum_required(VERSION 2.8.3)
project(panoptic_mapping)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)

catkin_package()

cs_add_executable(panoptic_mapper
        src/ros_nodes/panoptic_mapper_node.cpp
        )
target_link_libraries(panoptic_mapper ${catkin_LIBRARIES})
