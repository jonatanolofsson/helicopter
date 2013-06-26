cmake_minimum_required(VERSION 2.8)
include(../app.cmake)
app(servotest)

local_modules(
    states
    servomover
)

shared_modules(
   com
   clock
   math
)

executable_app(main.cpp)

