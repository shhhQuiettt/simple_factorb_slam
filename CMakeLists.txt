cmake_minimum_required(VERSION 3.16)

project(SimSlamProject LANGUAGES C CXX)

find_program(CCACHE_PROGRAM ccache)

if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
    set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
else()
    message(STATUS "ccache not found, not using it")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(shared_includes INTERFACE)
target_include_directories(shared_includes INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(project_options INTERFACE)
target_compile_options(project_options INTERFACE -Wall -Wextra -Wpedantic -Winvalid-pch -gsplit-dwarf)

# ================ #
 
find_package(raylib REQUIRED)

set(SIM_SOURCES src/simulator/main.c
    src/simulator/make_measurements.c
)


add_executable(simulator
    ${SIM_SOURCES}
)

target_link_libraries(simulator 
    PRIVATE 
        raylib
        shared_includes
        project_options
)

# ================ #

find_package(GTSAM REQUIRED)

set(SLAM_SOURCES src/slam/main.cpp)

add_executable(slam_node
    ${SLAM_SOURCES}
)

target_link_libraries(slam_node 
    PRIVATE 
        gtsam
        shared_includes
        project_options
)

target_precompile_headers(slam_node PRIVATE
    <vector>
    <memory>
    <gtsam/geometry/Pose3.h>
    <gtsam/inference/Symbol.h>              
    <gtsam/navigation/GPSFactor.h>
    <gtsam/nonlinear/ISAM2.h>
    <gtsam/nonlinear/NonlinearFactorGraph.h>
    <gtsam/nonlinear/Values.h>
    <gtsam/slam/BearingRangeFactor.h>
    <gtsam/slam/BetweenFactor.h>
    <gtsam/nonlinear/Marginals.h>
    <gtsam/slam/PriorFactor.h>
)
