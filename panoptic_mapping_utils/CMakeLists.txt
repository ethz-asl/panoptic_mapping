cmake_minimum_required(VERSION 2.8.3)
project(panoptic_mapping_utils)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
        src/evaluation/map_evaluator.cpp
        )

###############
# Executables #
###############

cs_add_executable(single_map_evaluation
        app/single_map_evaluation.cpp
        )
target_link_libraries(single_map_evaluation ${PROJECT_NAME})

cs_add_executable(multi_map_evaluation
        app/multi_map_evaluation.cpp
        )
target_link_libraries(multi_map_evaluation ${PROJECT_NAME})

##########
# Export #
##########

cs_install()
cs_export()

