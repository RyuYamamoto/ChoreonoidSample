cmake_minimum_required(VERSION 2.6)

#project(get_link_info)

include(FindPkgConfig)
include(CheckIncludeFiles)

set(CMAKE_CXX_FLAGS "-std=c++11")

pkg_check_modules(CHOREONOID REQUIRED choreonoid-body-plugin)
include_directories(${CHOREONOID_INCLUDE_DIRS})
link_directories(${CHOREONOID_LIBRARY_DIRS})

add_executable(get_link_info get_link_info.cpp)
target_link_libraries(get_link_info ${CHOREONOID_LIBRARIES})

add_executable(kinematics kinematics.cpp)
target_link_libraries(kinematics ${CHOREONOID_LIBRARIES})
