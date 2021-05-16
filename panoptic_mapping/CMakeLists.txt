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
        src/common/camera.cpp
        src/common/input_data_user.cpp
        src/map/submap.cpp
        src/map/submap_collection.cpp
        src/map/utils/submap_id.cpp
        src/map/utils/instance_id.cpp
        src/map/utils/submap_bounding_volume.cpp
        src/preprocessing/label_handler.cpp
        src/preprocessing/tracking_info.cpp
        src/preprocessing/single_tsdf_tracker.cpp
        src/preprocessing/ground_truth_id_tracker.cpp
        src/preprocessing/projective_id_tracker.cpp
        src/preprocessing/detectron_id_tracker.cpp
        src/preprocessing/geometric_edge_refiner.cpp
        src/integrator/projective_integrator.cpp
        src/integrator/class_projective_integrator.cpp
        src/integrator/single_tsdf_integrator.cpp
        src/integrator/projection_interpolators.cpp
        src/integrator/mesh_integrator.cpp
        src/map_management/tsdf_registrator.cpp
        src/map_management/activity_manager.cpp
        src/tools/planning_interface.cpp
        src/tools/map_renderer.cpp
        src/tools/data_writer.cpp
        )
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_proto)

##########
# Export #
##########

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)