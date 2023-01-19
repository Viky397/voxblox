cmake_minimum_required(VERSION 2.8.3)
project(object_detection)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_compile_options(-std=c++14)

set(CMAKE_CFG_PATH ${CMAKE_CURRENT_LIST_DIR}/../../Kimera-Semantics/kimera_semantics_ros/cfg)
add_definitions(-DCFG_PATH="${CMAKE_CFG_PATH}")

set(ROSLINT_CPP_OPTS "--filter=-legal, -build/c++11, -whitespace/indent, -whitespace/braces, -runtime/references, -readability/casting")
roslint_cpp()

find_package(OpenCV 3.4.19 REQUIRED)

find_package(Boost REQUIRED COMPONENTS
    system
    filesystem
)

find_package(PkgConfig)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

cs_add_library(object_detection_lib
  src/object_tracker/object_tracker_node.cpp
  src/secondary_clustering/secondary_clustering_node.cpp
)
target_link_libraries(object_detection_lib
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

cs_install()
cs_export()

