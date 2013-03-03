#include <gtest/gtest.h>

#include <sys/math/models/models.hpp>
#include <sys/Observer/gps.hpp>
#include <sys/math/filtering/GaussianFilter.hpp>
#include <sys/math/filtering/EKF.hpp>
#include <Eigen/Core>


using namespace Eigen;
using namespace sys;

typedef math::models::SCart3DQuat states;
typedef math::models::NoControl controls;
typedef math::models::Description<states, controls> ModelDescription;
typedef math::GaussianFilter<ModelDescription> SystemState;
typedef models::motion::ConstantVelocities3D MotionModel;
typedef math::EKF Filter;

TEST(EKFTests, TimeUpdateStandStill) {
    SystemState system_state;
    states::initialize(system_state);
    auto reference = system_state.state;
    Filter::timeUpdate<MotionModel>(system_state, 1.0);
    EXPECT_EQ(reference, system_state.state) << reference.transpose() << " = " << system_state.state.transpose();
}

TEST(EKFTests, TimeUpdateMovingPositiveX) {
    SystemState system_state;
    states::initialize(system_state);
    system_state.state[states::vx] = 1.0;
    auto reference = system_state.state;
    reference[states::x] = 1.0;

    Filter::timeUpdate<MotionModel>(system_state, 1.0);
    EXPECT_EQ(reference, system_state.state) << reference.transpose() << " = " << system_state.state.transpose();
}

TEST(EKFTests, TimeUpdateMovingNegativeX) {
    SystemState system_state;
    states::initialize(system_state);
    system_state.state[states::vx] = -1.0;
    auto reference = system_state.state;
    reference[states::x] = -1.0;

    Filter::timeUpdate<MotionModel>(system_state, 1.0);
    EXPECT_EQ(reference, system_state.state) << reference.transpose() << " = " << system_state.state.transpose();
}

TEST(EKFTests, TimeUpdateRotationPositiveX) {
    SystemState system_state;
    states::initialize(system_state);
    system_state.state[states::wx] = 1.0;
    auto reference = system_state.state;
    reference[states::qw] = 0;
    reference[states::qx] = -1.0;

    Filter::timeUpdate<MotionModel>(system_state, M_PI);
    EXPECT_TRUE((reference-system_state.state).norm() < 1e-7) << reference.transpose() << " = " << system_state.state.transpose();
}

TEST(EKFTests, TimeUpdateRepeatedRotationPositiveX) {
    SystemState system_state;
    states::initialize(system_state);
    system_state.state[states::wx] = 1.0;
    auto reference = system_state.state;
    reference[states::qw] = 0;
    reference[states::qx] = -1.0;

    for(int i = 0; i < 100; ++i) {
        Filter::timeUpdate<MotionModel>(system_state, M_PI/100);
    }
    EXPECT_TRUE((reference-system_state.state).norm() < 1e-5) << reference.transpose() << " = " << system_state.state.transpose();
}


TEST(EKFTests, MeasurementUpdateGPS) {
    SystemState system_state;
    states::initialize(system_state);
    auto reference = system_state.state;
    reference[states::x] = reference[states::y] = reference[states::z] = 0.5;

    math::Measurement<observer::GPS<>> m;
    m.z << 1, 1, 1, 0, 0, 0;
    m.R.setIdentity();

    Filter::measurementUpdate<>(system_state, m);
    EXPECT_TRUE((reference-system_state.state).norm() < 1e-5) << reference.transpose() << " = " << system_state.state.transpose();
}
