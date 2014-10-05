#include <gtest/gtest.h>
#include <iostream>
#include <sys/math/control.hpp>
#include <sys/math/states.hpp>
#include <sys/math/models.hpp>
#include <sys/math/filtering.hpp>
#include <Eigen/Core>

using namespace Eigen;
using namespace sys;
typedef math::models::SCart3DQuat<> StateDescription;
typedef StateDescription states;
typedef math::GaussianFilter<StateDescription> Filter;
typedef math::models::ConstantVelocities6D<StateDescription> MotionModel;

IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

class LqTests : public ::testing::Test {
    public:
        typedef math::LqController<StateDescription> Controller;
        Controller controller;
        LqTests() : controller() {}
};


TEST_F(LqTests, OutputControl) {
    Filter filter;
    StateDescription::StateDescription::initialize(filter);
    StateDescription::Controls u; u.setZero();
    filter.state[states::x] = 10.0;

    controller.updateModel<MotionModel::isDiscrete>(MotionModel::systemJacobian(filter.state, u), MotionModel::controlJacobian(filter.state, u));
    controller(filter.state);
}


TEST_F(LqTests, StressTest3D) {
    Filter filter;
    StateDescription::StateDescription::initialize(filter);
    filter.state[states::x] = 10.0;
    StateDescription::Controls u; u.setZero();

    for(int i = 0; i < 10000; ++i) {
        controller.updateModel<MotionModel::isDiscrete>(MotionModel::systemJacobian(filter.state, u), MotionModel::controlJacobian(filter.state, u));
        controller(filter.state);
    }
}
