project(sys)
find_package(Eigen3)
include_directories (${Eigen3_INCLUDE_DIR})
include_directories(include)
list(APPEND CMAKE_CXX_FLAGS "-std=c++0x")

add_subdirectory(observer)
add_subdirectory(math)
