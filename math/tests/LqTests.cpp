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
typedef math::GaussianFilter<ModelDescription> SystemState;
typedef models::motion::DirectVelocities3D MotionModel;

IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

class LqTests : public ::testing::Test {
    public:
        typedef math::LqController<ModelDescription> Controller;
        Controller controller;
        LqTests() : controller() {}
};


TEST_F(LqTests, OutputControl) {
    SystemState systemState;
    ModelDescription::StateDescription::initialize(systemState);
    systemState.state[states::x] = 10.0;

    controller.updateModel<MotionModel::CD>(MotionModel::systemJacobian<>(systemState), MotionModel::controlJacobian<>(systemState));
    controller(systemState.state);
}


TEST_F(LqTests, StressTest3D) {
    SystemState systemState;
    ModelDescription::StateDescription::initialize(systemState);
    systemState.state[states::x] = 10.0;

    for(int i = 0; i < 10000; ++i) {
        controller.updateModel<MotionModel::CD>(MotionModel::systemJacobian<>(systemState), MotionModel::controlJacobian<>(systemState));
        controller(systemState.state);
    }
}
