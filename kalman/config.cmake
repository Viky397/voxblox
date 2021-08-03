cmake_minimum_required(VERSION 3.1)

if (NOT PROJECT_NAME)
    message(WARNING "Project name is unset! This file should only be included after calling project(...)")
endif(NOT PROJECT_NAME)

if (NOT GLOBAL_CONFIG_SET)
    set(GLOBAL_CONFIG_SET TRUE)
    set(CMAKE_VERBOSE_MAKEFILE TRUE)
    set(CMAKE_CXX_STANDARD 14)
    if (IS_CMAKE_BUILD)
        list(APPEND CMAKE_PREFIX_PATH "/opt/ros/kinetic")
    endif(IS_CMAKE_BUILD)
    set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
    if (CMAKE_BUILD_TYPE MATCHES "Debug")
        message(STATUS "ZEUS_DEBUG flag is enabled.")
        add_definitions(-DZEUS_DEBUG)
    endif(CMAKE_BUILD_TYPE MATCHES "Debug")

    set(DEFENSIVE_FLAGS "-D_GLIBCXX_ASSERTIONS -fstack-protector-strong -fexceptions") # -D_FORTIFY_SOURCE=2 would enable runtime overflow checks, but requires O2
    if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 8.0)
        set(DEFENSIVE_FLAGS "${DEFENSIVE_FLAGS} -fstack-clash-protection -fcf-protection")
    endif()
    set(DEBUGGING_FLAGS "-fasynchronous-unwind-tables -grecord-gcc-switches")
    set(WARNING_FLAGS "-Wall -W -Wno-unused-parameter -Werror=format-security -Werror=implicit-function-declaration -Wl,-z,now -Wl,-z,relro") #  -Wl,-z,defs would prevent underlinking, but doesn't play nice with ROS/Catkin
    set(OPTIMIZATION_FLAGS "-pipe -Og") # -O2 is required for -D_FORTIFY_SOURCE=2, but would obfuscate debugging
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEFENSIVE_FLAGS} ${DEBUGGING_FLAGS} ${WARNING_FLAGS} ${OPTIMIZATION_FLAGS}")
endif()

if (NOT ${PROJECT_NAME}_CONFIG_SET)
    set(${PROJECT_NAME}_CONFIG_SET TRUE)
else(NOT ${PROJECT_NAME}_CONFIG_SET)
    message(WARNING "config.cmake was included twice for the same project - make sure that the `include(config.cmake)` call happens after calling project(...).")
endif(NOT ${PROJECT_NAME}_CONFIG_SET)
