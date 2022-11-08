cmake_minimum_required(VERSION 3.0.2)
project(panoptic_mapping)

set(CMAKE_CXX_STANDARD 17)
# add_definitions(-Wall)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

############
# PROTOBUF #
############

set(PROTO_DEFNS
        proto/panoptic_mapping/Submap.proto
        proto/panoptic_mapping/SubmapCollection.proto)
set(BASE_PATH "proto")
PROTOBUF_CATKIN_GENERATE_CPP2(${BASE_PATH} PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

cs_add_library(${PROJECT_NAME}_proto ${PROTO_SRCS})
target_link_libraries(${PROJECT_NAME}_proto ${PROTOBUF_LIBRARIES})

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
        src/common/camera.cpp
        src/common/input_data_user.cpp
        src/map/submap.cpp
        src/map/submap_collection.cpp
        src/map/submap_id.cpp
        src/map/instance_id.cpp
        src/map/submap_bounding_volume.cpp
        src/map/classification/binary_count.cpp
        src/map/classification/moving_binary_count.cpp
        src/map/classification/fixed_count.cpp
        src/map/classification/variable_count.cpp
        src/map/classification/uncertainty.cpp
        src/map/scores/average.cpp
        src/map/scores/latest.cpp
        src/labels/label_handler_base.cpp
        src/labels/null_label_handler.cpp
        src/labels/csv_label_handler.cpp
        src/labels/range_label_handler.cpp
        src/tracking/tracking_info.cpp
        src/tracking/single_tsdf_tracker.cpp
        src/tracking/ground_truth_id_tracker.cpp
        src/tracking/projective_id_tracker.cpp
        src/tracking/detectron_id_tracker.cpp
        src/submap_allocation/null_submap_allocator.cpp
        src/submap_allocation/semantic_submap_allocator.cpp
        src/submap_allocation/monolithic_freespace_allocator.cpp
        src/integration/projective_tsdf_integrator.cpp
        src/integration/class_projective_tsdf_integrator.cpp
        src/integration/single_tsdf_integrator.cpp
        src/integration/projection_interpolators.cpp
        src/integration/mesh_integrator.cpp
        src/map_management/map_manager.cpp
        src/map_management/null_map_manager.cpp
        src/map_management/activity_manager.cpp
        src/map_management/tsdf_registrator.cpp
        src/map_management/layer_manipulator.cpp
        src/tools/planning_interface.cpp
        src/tools/map_renderer.cpp
        src/tools/null_data_writer.cpp
        src/tools/log_data_writer.cpp
        src/tools/evaluation_data_writer.cpp
        src/tools/serialization.cpp
        )
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_proto stdc++fs)

##########
# Tests #
##########

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(serialization-test test/serialization.cpp)
    target_link_libraries(serialization-test ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()

##########
# Export #
##########

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)
