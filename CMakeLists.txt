cmake_minimum_required(VERSION 2.8)
project(Heliosys)
include(common/CMake/common.cmake)
include_directories(.)
find_package(Boost COMPONENTS system)

add_definitions(-std=c++0x -Wall -Werror -pedantic-errors -Wextra -Wcast-align -ggdb -O0 -Wfatal-errors)

add_subdirectory(common)
add_subdirectory(os)
add_subdirectory(sys)
