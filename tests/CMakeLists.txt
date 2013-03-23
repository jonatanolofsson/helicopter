make_test(ActuatorTests)
target_link_libraries(ActuatorTests actuator os)
make_test(SensorTests)
make_test(ControllerTests)

add_executable(ImuPrint ImuPrint.cpp)
target_link_libraries(ImuPrint pthread cpplot)
