project(sys)
find_package(Eigen3)
include_directories(${Eigen3_INCLUDE_DIR})

add_subdirectory(math)
add_subdirectory(tests)
#~ add_subdirectory(states)

#~ add_executable(heli main.cpp)
#~ target_link_libraries(heli math states)
