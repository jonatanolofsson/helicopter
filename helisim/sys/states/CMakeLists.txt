add_library(states
    states.cpp
    Helicopter.cpp
    Fly.cpp
)

target_link_libraries(states
    observer
)
