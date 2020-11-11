cmake_minimum_required(VERSION 2.8.3)
project(panoptic_mapping)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

############
# PROTOBUF #
############
# Taken from voxblox
set(PROTO_DEFNS
        proto/panoptic_mapping/Submap.proto
        proto/panoptic_mapping/SubmapCollection.proto)
set(BASE_PATH "proto")
PROTOBUF_CATKIN_GENERATE_CPP2(${BASE_PATH} PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

#############
# Libraries #
#############
# Single Protobuf Library
cs_add_library(${PROJECT_NAME}_proto ${PROTO_SRCS})
target_link_libraries(${PROJECT_NAME}_proto ${PROTOBUF_LIBRARIES})

cs_add_library(${PROJECT_NAME}
        src/core/submap.cpp
        src/core/submap_id.cpp
        src/core/submap_bounding_volume.cpp
        src/core/submap_collection.cpp
        src/preprocessing/label_handler.cpp
        src/preprocessing/ground_truth_id_tracker.cpp
        src/integrator/integrator_base.cpp
        src/integrator/naive_integrator.cpp
        src/integrator/projective_integrator.cpp
        src/integrator/projection_interpolators.cpp
        src/registration/tsdf_registrator.cpp
        )
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_proto)

##########
# Export #
##########

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)