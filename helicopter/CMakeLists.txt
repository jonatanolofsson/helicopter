cmake_minimum_required(VERSION 2.8)
include(../app.cmake)
app(helicopter)

local_modules(
    states
    sensorhub
)

shared_modules(
   motioncontrol
   actuator
   observer
   com
   clock
   math
)

executable_app(main.cpp)

