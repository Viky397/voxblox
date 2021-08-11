cmake_minimum_required(VERSION 2.8.3)
project(custom_vis)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  geodesy
  tf2_geometry_msgs
  eigen_conversions
  tf_conversions
  pcl_ros
  zeus_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs geodesy tf2_geometry_msgs visualization_msgs
)


include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(revis_node
    src/revis_node.cpp
)

target_link_libraries(revis_node
  ${catkin_LIBRARIES}
)

