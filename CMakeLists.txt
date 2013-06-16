find_package(Eigen3)
include_directories(${Eigen3_INCLUDE_DIR})

set(libraries
    math
    com
    motioncontrol
    actuator
    observer
    clock
)

foreach(lib ${libraries})
    add_subdirectory(${lib}/tests)
endforeach()

add_subdirectory(tests)
