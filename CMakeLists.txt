find_package(Eigen3)
include_directories(${Eigen3_INCLUDE_DIR})

set(libraries
    math
    com
    motioncontrol
    actuator
    observer
    simulator
    localplanner
    sensorhub
    states
    clock
)

foreach(lib ${libraries})
    add_subdirectory(${lib})
endforeach()

add_executable(firefighter main.cpp)
target_link_libraries(firefighter ${libraries} ${libraries} os pthread)

add_subdirectory(tests)
