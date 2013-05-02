add_library(states
    Top.cpp
    states.cpp
    Firefighter.cpp
)

add_subdirectory(moving)

target_link_libraries(states
    states_moving
    observer
)
