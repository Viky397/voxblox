cmake_minimum_required(VERSION 2.8.3)
project(voxblox_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_compile_options(-std=c++14)

set(CMAKE_CFG_PATH ${CMAKE_CURRENT_LIST_DIR}/../../Kimera-Semantics/kimera_semantics_ros/cfg)
add_definitions(-DCFG_PATH="${CMAKE_CFG_PATH}")

#############
# LIBRARIES #
#############
cs_add_library(voxblox_ros_lib
  src/esdf_server.cc
  src/interactive_slider.cc
  src/simulation_server.cc
  src/transformer.cc
  src/tsdf_server.cc
  src/pocd_tsdf_server.cc
)

##########
# EXPORT #
##########
cs_install()
cs_export()
