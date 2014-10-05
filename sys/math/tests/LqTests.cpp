#include <gtest/gtest.h>
#include <iostream>
#include <sys/math/control.hpp>
#include <sys/math/states.hpp>
#include <sys/math/models.hpp>
#include <sys/math/filtering.hpp>
#include <Eigen/Core>

using namespace Eigen;
using namespace sys;
typedef math::models::SCart3D states;
typedef math::models::CVel3 controls;
typedef math::models::Description<states, controls> ModelDescription;
typedef math::GaussianFilter<ModelDescription> Filter;
typedef math::models::DirectVelocities3D<ModelDescription> MotionModel;

IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

class LqTests : public ::testing::Test {
    public:
        typedef math::LqController<ModelDescription> Controller;
        Controller controller;
        LqTests() : controller() {}
};


TEST_F(LqTests, OutputControl) {
    Filter filter;
    ModelDescription::StateDescription::initialize(filter);
    ModelDescription::Controls u; u.setZero();
    filter.state[states::x] = 10.0;

    controller.updateModel<MotionModel::isDiscrete>(MotionModel::systemJacobian(filter.state, u), MotionModel::controlJacobian(filter.state, u));
    controller(filter.state);
}


TEST_F(LqTests, StressTest3D) {
    Filter filter;
    ModelDescription::StateDescription::initialize(filter);
    filter.state[states::x] = 10.0;
    ModelDescription::Controls u; u.setZero();

    for(int i = 0; i < 10000; ++i) {
        controller.updateModel<MotionModel::isDiscrete>(MotionModel::systemJacobian(filter.state, u), MotionModel::controlJacobian(filter.state, u));
        controller(filter.state);
    }
}
