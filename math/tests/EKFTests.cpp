#include <gtest/gtest.h>

#include <sys/math/models/models.hpp>
#include <sys/Observer/Observer.hpp>
#include <sys/Observer/gps.hpp>
#include <sys/math/filtering/KalmanFilter.hpp>
#include <sys/math/filtering/ExtendedKalmanFilter.hpp>

using namespace sys;

typedef math::S6DOFQ StateDescription;
typedef StateDescription states;
typedef models::motion::ConstantVelocities  MotionModel;
typedef math::ExtendedKalmanFilter filter;
typedef math::KalmanFilter<StateDescription> SystemState;

TEST(EKFTests, TimeUpdateStandStill) {
    SystemState system_state(1.0);
    StateDescription::initialize(system_state);
    auto reference = system_state.state;
    filter::timeUpdate<MotionModel>(system_state);
    EXPECT_EQ(reference, system_state.state) << reference.transpose() << " = " << system_state.state.transpose();
}

TEST(EKFTests, TimeUpdateMovingPositiveX) {
    SystemState system_state(1.0);
    StateDescription::initialize(system_state);
    system_state.state[states::vx] = 1.0;
    auto reference = system_state.state;
    reference[states::x] = 1.0;

    filter::timeUpdate<MotionModel>(system_state);
    EXPECT_EQ(reference, system_state.state) << reference.transpose() << " = " << system_state.state.transpose();
}

TEST(EKFTests, TimeUpdateMovingNegativeX) {
    SystemState system_state(1.0);
    StateDescription::initialize(system_state);
    system_state.state[states::vx] = -1.0;
    auto reference = system_state.state;
    reference[states::x] = -1.0;

    filter::timeUpdate<MotionModel>(system_state);
    EXPECT_EQ(reference, system_state.state) << reference.transpose() << " = " << system_state.state.transpose();
}

TEST(EKFTests, TimeUpdateRotationPositiveX) {
    SystemState system_state(M_PI);
    StateDescription::initialize(system_state);
    system_state.state[states::wx] = 1.0;
    auto reference = system_state.state;
    reference[states::qw] = 0;
    reference[states::qx] = -1.0;

    filter::timeUpdate<MotionModel>(system_state);
    EXPECT_TRUE((reference-system_state.state).norm() < 1e-7) << reference.transpose() << " = " << system_state.state.transpose();
}

TEST(EKFTests, TimeUpdateRepeatedRotationPositiveX) {
    SystemState system_state(M_PI/100);
    StateDescription::initialize(system_state);
    system_state.state[states::wx] = 1.0;
    auto reference = system_state.state;
    reference[states::qw] = 0;
    reference[states::qx] = -1.0;

    for(int i = 0; i < 100; ++i) {
        filter::timeUpdate<MotionModel>(system_state);
    }
    EXPECT_TRUE((reference-system_state.state).norm() < 1e-5) << reference.transpose() << " = " << system_state.state.transpose();
}


TEST(EKFTests, MeasurementUpdateGPS) {
    SystemState system_state(M_PI/100);
    StateDescription::initialize(system_state);
    auto reference = system_state.state;
    reference[states::x] = reference[states::y] = reference[states::z] = 0.5;

    using namespace observer;

    Measurement<GPS<>> m;
    m.z << 1, 1, 1, 0, 0, 0;
    m.R.setIdentity();

    filter::measurementUpdate<StateDescription>(system_state, m);
    EXPECT_TRUE((reference-system_state.state).norm() < 1e-5) << reference.transpose() << " = " << system_state.state.transpose();
}
