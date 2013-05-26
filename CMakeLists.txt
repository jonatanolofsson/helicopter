find_package(Eigen3)
include_directories(${Eigen3_INCLUDE_DIR})

set(libraries
    math
    com
    motioncontrol
    actuator
    observer
    sensorhub
    states
    clock
)

foreach(lib ${libraries})
    add_subdirectory(${lib})
endforeach()

add_executable(helicopter main.cpp)
target_link_libraries(helicopter ${libraries} ${libraries} os pthread)

add_subdirectory(tests)
