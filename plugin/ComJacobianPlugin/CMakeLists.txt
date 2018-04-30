cmake_minimum_required(VERSION 2.8)

project(ComJacobianPlugin)

find_package(Boost COMPONENTS system thread filesystem)

add_definitions(-std=c++11)

include(FindPkgConfig)
pkg_check_modules(cnoid_plugin REQUIRED choreonoid-body-plugin)

include_directories(
	${cnoid_plugin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

link_directories(
	${cnoid_plugin_LIBRARY_DIRS}
	${Boost_LIBRARY_DIRS}
)

add_library(ComJacobianPlugin SHARED ComJacobianPlugin.cpp)
set_target_properties(ComJacobianPlugin PROPERTIES PREFIX "")

set(target ComJacobianPlugin)
target_link_libraries(ComJacobianPlugin
	${cnoid_plugin_LIBRARIES}
	${Boost_LIBRARIES}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

install(TARGETS ComJacobianPlugin LIBRARY DESTINATION /usr/lib/choreonoid-1.6/)
install(FILES pdgain.txt DESTINATION /usr/lib/choreonoid-1.6/)
