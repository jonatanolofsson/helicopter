add_library(states
    states.cpp
    Top.cpp
    Helicopter.cpp
)

target_link_libraries(states
    observer
)
