add_library(simulator
    instantiation.cpp
)
target_link_libraries(simulator math)

add_subdirectory(tests)

