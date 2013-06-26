cmake_minimum_required(VERSION 2.8)
include(../app.cmake)
app(helicopter)

local_modules(
    states
)

shared_modules(
    motioncontrol
    simulator
    referencegenerator
    sensorhub
    observer
    clock
    logger
    math
)

executable_app(main.cpp)

