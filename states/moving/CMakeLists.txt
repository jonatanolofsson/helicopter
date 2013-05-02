add_library(states_moving
    Moving.cpp
    InitializePosition.cpp
)

add_subdirectory(running)

target_link_libraries(states_moving
    states_running
    actuator
    particlefilter
)
