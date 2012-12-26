project(Observer)

make_module(
    Observer.cpp
    gps.cpp
)

add_subdirectory(tests)

