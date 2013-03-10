add_library(states
    Top.cpp
)

add_subdirectory(moving)

target_link_libraries(states
    states_moving
    observer
)
