cmake_minimum_required(VERSION 2.6)

project(model)

include(FindPkgConfig)
include(CheckIncludeFiles)

set(CMAKE_CXX_FLAGS "-std=c++11")

pkg_check_modules(CHOREONOID REQUIRED choreonoid-body-plugin)
include_directories(${CHOREONOID_INCLUDE_DIRS})
link_directories(${CHOREONOID_LIBRARY_DIRS})

add_executable(model test.cpp)

target_link_libraries(model ${CHOREONOID_LIBRARIES})
