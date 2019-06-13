cmake_minimum_required(VERSION 2.6)
include(FindPkgConfig)
include(CheckIncludeFiles)

pkg_check_modules(CHOREONOID REQUIRED choreonoid-body-plugin)
include_directories(${CHOREONOID_INCLUDE_DIRS})
link_directories(${CHOREONOID_LIBRARY_DIRS})
include_directories(${CMAKE_MODULE_PATH})

add_definitions(-std=c++11) 
add_executable(1link 1link.cpp)
target_link_libraries(1link ${CHOREONOID_LIBRARIES})


