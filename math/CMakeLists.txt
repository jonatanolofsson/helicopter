add_subdirectory(models)
add_library(math)
target_link_libraries(math models)
add_subdirectory(tests)
