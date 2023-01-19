cmake_minimum_required(VERSION 2.8.3)
project(zeus_pcl)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

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

cs_add_library(zeus_pcl_lib
    src/utils/zeus_pcl.cpp)
target_link_libraries(zeus_pcl_lib
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
add_dependencies(zeus_pcl_lib zeus_msgs_generate_messages_cpp)

cs_install()
cs_export()
