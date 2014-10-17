#include <gtest/gtest.h>
#include <iostream>
#include <sys/math/control.hpp>
#include <sys/math/states.hpp>
#include <sys/math/models.hpp>
#include <sys/math/filtering.hpp>
#include <Eigen/Core>

using namespace Eigen;
using namespace sys;
typedef math::models::SCart3DQuat<> States;
typedef math::GaussianFilter<StateDescription> Filter;
typedef math::models::ConstantVelocities6D MotionModel;

IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

class LqTests : public ::testing::Test {
    public:
        typedef math::LqController<States> Controller;
        Controller controller;
        LqTests() : controller() {}
};


TEST_F(LqTests, OutputControl) {
    Filter filter;
    States::initialize(filter);
    filter.state[States::x] = 10.0;

    controller.updateModel<MotionModel::isDiscrete>(
        math::differentiate<MotionModel::States, States>(filter.state),
        math::differentiate<Controls<MotionModel>, States>(filter.state));
    controller(filter.state);
}


TEST_F(LqTests, StressTest3D) {
    Filter filter;
    StateDescription::initialize(filter);
    filter.state[States::x] = 10.0;

    for(int i = 0; i < 10000; ++i) {
        controller.updateModel<MotionModel::isDiscrete>(MotionModel::systemJacobian(filter.state, u), MotionModel::controlJacobian(filter.state));
        controller(filter.state);
    }
}
