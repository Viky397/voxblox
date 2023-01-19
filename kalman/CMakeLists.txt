cmake_minimum_required(VERSION 2.8.3)
project(kalman)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(ROSLINT_CPP_OPTS "--filter=-legal, -build/c++11, -whitespace/indent, -whitespace/braces, -runtime/references, -readability/casting")
roslint_cpp()

find_package(OpenCV 3.4.19 REQUIRED)

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(PkgConfig REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(KALMAN_UTILS_SRC
  src/utils/kalman.cpp
  src/utils/association.cpp
  src/utils/transform_utils.cpp
  src/utils/hmm.cpp
  src/types/objects3D.cpp
  src/types/Pose2.cpp
)

cs_add_library(kalman_lib
    ${KALMAN_UTILS_SRC}
)
target_link_libraries(kalman_lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

cs_install()
cs_export()
