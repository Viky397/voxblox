cmake_minimum_required(VERSION 2.8.3)
project(kalman)
include("./config.cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  zeus_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf
  geodesy
  zeus_pcl
  roslint
  pcl_conversions
  pcl_ros
)

set(ROSLINT_CPP_OPTS "--filter=-legal, -build/c++11, -whitespace/indent, -whitespace/braces, -runtime/references, -readability/casting")
roslint_cpp()

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(PkgConfig REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kalman_lib
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf geodesy tf2_geometry_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(KALMAN_UTILS_SRC
  src/utils/kalman.cpp
  src/utils/association.cpp
  src/utils/transform_utils.cpp
  src/utils/hmm.cpp
  src/types/objects3D.cpp
  src/types/Pose2.cpp
)

add_library(kalman_lib
    ${KALMAN_UTILS_SRC}
)
target_link_libraries(kalman_lib ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS})
