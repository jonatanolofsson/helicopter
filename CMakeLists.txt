find_package(Eigen3)
include_directories(${Eigen3_INCLUDE_DIR})

set(libraries
    motioncontrol
    actuator
    observer
    states
    com
    math
)

foreach(lib ${libraries})
    add_subdirectory(${lib})
endforeach()

add_executable(firefighter main.cpp)
target_link_libraries(firefighter ${libraries})

add_subdirectory(tests)
