cmake_minimum_required(VERSION 3.15)

project(SimSlamProject LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(shared_includes INTERFACE)
target_include_directories(shared_includes INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ================ #
 
find_package(raylib REQUIRED)

file(GLOB_RECURSE SIM_SOURCES CONFIGURE_DEPENDS src/simulator/*.c)

add_executable(simulator
    ${SIM_SOURCES}
)

target_link_libraries(simulator 
    PRIVATE 
        raylib
        shared_includes
)

# ================ #

find_package(GTSAM REQUIRED)

file(GLOB_RECURSE SLAM_SOURCES CONFIGURE_DEPENDS src/slam/*.cpp)

add_executable(slam_node
    ${SLAM_SOURCES}
)

target_link_libraries(slam_node 
    PRIVATE 
        gtsam
        shared_includes
)
