cmake_minimum_required(VERSION 2.8.3)
project(panoptic_mapping_ros)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
        src/panoptic_mapper.cpp
        src/visualization/tsdf_visualizer.cpp
        src/conversions/ros_params.cpp
        )

###############
# Executables #
###############

cs_add_executable(panoptic_mapper_node
        src/ros_nodes/panoptic_mapper_node.cpp
        )
target_link_libraries(panoptic_mapper_node ${PROJECT_NAME})

