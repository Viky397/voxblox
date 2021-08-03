cmake_minimum_required(VERSION 2.8.3)
project(zeus_msgs)
include(./config.cmake)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    tf2
    tf2_geometry_msgs
    roslint
)

add_message_files( DIRECTORY msg FILES
    BoundingBox3D.msg
    Detections3D.msg
    BoundingBox2D.msg
    Detections2D.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS
    message_runtime
    message_generation
    std_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    tf2
    tf2_geometry_msgs
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
)
