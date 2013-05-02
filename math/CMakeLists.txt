add_subdirectory(states)
add_library(math
)
target_link_libraries(math models)
add_subdirectory(tests)
