cmake_minimum_required(VERSION 2.8.3)
project(panoptic_mapping)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)

catkin_package()

############
# PROTOBUF #
############
# Taken from voxblox
set(PROTO_DEFNS
        proto/panoptic_mapping/QuatTransformation.proto
        proto/panoptic_mapping/Submap.proto
        proto/panoptic_mapping/SubmapCollection.proto)
set(BASE_PATH "proto")
#set(PROTOBUF_COMPILE_PYTHON true)

PROTOBUF_CATKIN_GENERATE_CPP2(${BASE_PATH} PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

#############
# Libraries #
#############
# Single Protobuf Library
cs_add_library(${PROJECT_NAME}_proto ${PROTO_SRCS})
target_link_libraries(${PROJECT_NAME}_proto ${PROTOBUF_LIBRARIES})

cs_add_library(${PROJECT_NAME}
        src/panoptic_mapper.cpp
        src/core/submap.cpp
        src/core/submap_collection.cpp
        src/input/label_handler.cpp
        src/input/pointcloud_integrator_factory.cpp
        src/input/naive_integrator.cpp
        #src/input/pointcloud_integrator.cpp
        src/visualization/tsdf_visualizer.cpp
        )
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_proto)

# Executables
cs_add_executable(panoptic_mapper_node
        src/ros_nodes/panoptic_mapper_node.cpp
        )
target_link_libraries(panoptic_mapper_node ${PROJECT_NAME})

# Export
cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)