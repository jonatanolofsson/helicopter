#include <gtest/gtest.h>
#include <iostream>
#include <sys/math/control.hpp>
#include <sys/math/states.hpp>
#include <sys/math/models.hpp>
#include <sys/math/filtering.hpp>
#include <Eigen/Core>

using namespace Eigen;
using namespace sys;
typedef math::models::SCart3DQuat States;
typedef math::models::CVW Controls;
typedef math::GaussianFilter<States> Filter;
typedef math::models::ConstantVelocities6D MotionModel;
typedef math::LqController<States, Controls> Controller;

IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

class LqTests : public ::testing::Test {
    public:
        Controller controller;
        LqTests() : controller() {}
};


TEST_F(LqTests, OutputControl) {
    Filter filter;
    States::initialize(filter);
    filter.state[States::x] = 10.0;

    controller.updateModel<MotionModel::isDiscrete>(
        math::differentiate<MotionModel, States, Filter::States>(filter.state),
        math::differentiate<MotionModel, Controls, Filter::States>(filter.state));
    controller(filter.state);
}


TEST_F(LqTests, StressTest3D) {
    Filter filter;
    States::initialize(filter);
    filter.state[States::x] = 10.0;

    for(int i = 0; i < 10000; ++i) {
        controller.updateModel<MotionModel::isDiscrete>(
            math::differentiate<MotionModel, States, Filter::States>(filter.state),
            math::differentiate<MotionModel, Controls, Filter::States>(filter.state));
        controller(filter.state);
    }
}
