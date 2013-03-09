find_package(Eigen3)
include_directories(${Eigen3_INCLUDE_DIR})

set(libraries
    math
    states
    actuator
    observer
    motioncontrol
)

foreach(lib ${libraries})
    add_subdirectory(${lib})
endforeach()

add_executable(heli main.cpp)
target_link_libraries(heli libraries)

add_subdirectory(tests)
