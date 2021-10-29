cmake_minimum_required(VERSION 2.8.3)
project(object_detection)
include("./config.cmake")
add_compile_options(-std=c++11)

set(CMAKE_CFG_PATH ${CMAKE_CURRENT_LIST_DIR}/../../Kimera-Semantics/kimera_semantics_ros/cfg)
add_definitions(-DCFG_PATH="${CMAKE_CFG_PATH}")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_geometry
  image_transport
  genmsg
  nav_msgs
  zeus_msgs
  kalman
  zeus_pcl
  tf2
  tf2_geometry_msgs
  tf2_ros
  roslint
  pcl_conversions
  pcl_ros
)

set(ROSLINT_CPP_OPTS "--filter=-legal, -build/c++11, -whitespace/indent, -whitespace/braces, -runtime/references, -readability/casting")
roslint_cpp()

find_package(OpenCV 3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS
    system
    filesystem
)
find_package(PkgConfig)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES object_detection_lib
  CATKIN_DEPENDS kalman zeus_pcl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(OBJECT_DETECTION_SRC
  src/object_tracker/object_tracker_node.cpp
  src/secondary_clustering/secondary_clustering_node.cpp
  src/object_tracker/test_prob.cpp
)

add_library(object_detection_lib
    ${OBJECT_DETECTION_SRC}
)
target_link_libraries(object_detection_lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# Secondary Clustering
add_executable(secondary_clustering src/secondary_clustering/secondary_clustering.cpp)
target_link_libraries(secondary_clustering
  object_detection_lib
)
add_dependencies(secondary_clustering zeus_msgs_generate_messages_cpp)

# Object Tracker
add_executable(object_tracker src/object_tracker/object_tracker.cpp)
target_link_libraries(object_tracker
  object_detection_lib
)
add_dependencies(object_tracker zeus_msgs_generate_messages_cpp)

# Probability Test
add_executable(test_prob src/object_tracker/test_prob.cpp)
target_link_libraries(test_prob
  object_detection_lib
)
add_dependencies(test_prob zeus_msgs_generate_messages_cpp)

