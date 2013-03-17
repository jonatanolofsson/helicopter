make_test(ActuatorTests)
target_link_libraries(ActuatorTests actuator os)
make_test(SensorTests)
make_test(ControllerTests)
