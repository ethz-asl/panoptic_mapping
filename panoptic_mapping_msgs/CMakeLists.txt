cmake_minimum_required(VERSION 2.8.3)
project(panoptic_mapping_msgs)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

###########
# Export #
###########

cs_install()
cs_export()
