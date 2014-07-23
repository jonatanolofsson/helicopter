find_package(Eigen3)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-isystem${Eigen3_INCLUDE_DIR}")

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
