#include <gtest/gtest.h>

#include <sys/math/models/models.hpp>
#include <sys/observer/observer.hpp>
#include <os/shared/ProtectedData.hpp>
#include <sys/math/filtering/KalmanFilter.hpp>
#include <cmath>
#include <Eigen/Core>

using namespace sys;
using namespace Eigen;

typedef state_description::S6DOFQ StateDescription;
typedef StateDescription states;
typedef math::KalmanFilter<StateDescription> KF;
typedef models::motion::ConstantVelocities  MotionModel;

void initializeObserver(KF& system_state) {
    StateDescription::initialize(system_state);
}

TEST(ConstantVelocity, SingleDimension) {
    KF system_state(1.0);
    initializeObserver(system_state);
    system_state.state[states::vx] = 1.0;
    EXPECT_FLOAT_EQ(system_state.state[states::x], 0.0);
    EXPECT_FLOAT_EQ(system_state.state[states::qw], 1.0);

    auto x = MotionModel::predict<>(system_state);
    EXPECT_FLOAT_EQ(x[states::x], 1.0);
    EXPECT_FLOAT_EQ(x[states::vx], 1.0);
    EXPECT_FLOAT_EQ(x[states::y], 0.0);
    EXPECT_FLOAT_EQ(x[states::vy], 0.0);
}

TEST(ConstantVelocity, Diagonal) {
    KF system_state(1.0);
    initializeObserver(system_state);
    system_state.state[states::vx] = 1.0;
    system_state.state[states::vy] = 1.0;
    system_state.state[states::vz] = 1.0;
    EXPECT_FLOAT_EQ(system_state.state[states::x], 0.0);
    EXPECT_FLOAT_EQ(system_state.state[states::qw], 1.0);

    auto x = MotionModel::predict<>(system_state);
    EXPECT_FLOAT_EQ(x[states::x], 1.0);
    EXPECT_FLOAT_EQ(x[states::vx], 1.0);
    EXPECT_FLOAT_EQ(x[states::y], 1.0);
    EXPECT_FLOAT_EQ(x[states::vy], 1.0);
    EXPECT_FLOAT_EQ(x[states::z], 1.0);
    EXPECT_FLOAT_EQ(x[states::vz], 1.0);

    EXPECT_FLOAT_EQ(x[states::qx], 0.0);
    EXPECT_FLOAT_EQ(x[states::qy], 0.0);
    EXPECT_FLOAT_EQ(x[states::qz], 0.0);
    EXPECT_FLOAT_EQ(x[states::qw], 1.0);
}

TEST(ConstantVelocity, RotateX) {
    KF system_state(M_PI);
    initializeObserver(system_state);
    system_state.state[states::wx] = 1.0;

    auto x = MotionModel::predict<>(system_state);
    EXPECT_FLOAT_EQ(x[states::x], 0.0);
    EXPECT_FLOAT_EQ(x[states::vx], 0.0);
    EXPECT_FLOAT_EQ(x[states::y], 0.0);
    EXPECT_FLOAT_EQ(x[states::vy], 0.0);
    EXPECT_FLOAT_EQ(x[states::z], 0.0);
    EXPECT_FLOAT_EQ(x[states::vz], 0.0);

    EXPECT_FLOAT_EQ(x[states::qx], -1.0);
    EXPECT_FLOAT_EQ(x[states::qy], 0.0);
    EXPECT_FLOAT_EQ(x[states::qz], 0.0);
    EXPECT_NEAR(x[states::qw], 0.0, 1e-7);
}

TEST(ConstantVelocity, RotateY) {
    KF system_state(M_PI);
    initializeObserver(system_state);
    system_state.state[states::wy] = 1.0;

    auto x = MotionModel::predict<>(system_state);
    EXPECT_FLOAT_EQ(x[states::x], 0.0);
    EXPECT_FLOAT_EQ(x[states::vx], 0.0);
    EXPECT_FLOAT_EQ(x[states::y], 0.0);
    EXPECT_FLOAT_EQ(x[states::vy], 0.0);
    EXPECT_FLOAT_EQ(x[states::z], 0.0);
    EXPECT_FLOAT_EQ(x[states::vz], 0.0);

    EXPECT_FLOAT_EQ(x[states::qx], 0.0);
    EXPECT_FLOAT_EQ(x[states::qy], -1.0);
    EXPECT_FLOAT_EQ(x[states::qz], 0.0);
    EXPECT_NEAR(x[states::qw], 0.0, 1e-7);
}

TEST(ConstantVelocity, RotateZ) {
    KF system_state(M_PI);
    initializeObserver(system_state);
    system_state.state[states::wz] = -1.0;

    auto x = MotionModel::predict<>(system_state);
    EXPECT_FLOAT_EQ(x[states::x], 0.0);
    EXPECT_FLOAT_EQ(x[states::vx], 0.0);
    EXPECT_FLOAT_EQ(x[states::y], 0.0);
    EXPECT_FLOAT_EQ(x[states::vy], 0.0);
    EXPECT_FLOAT_EQ(x[states::z], 0.0);
    EXPECT_FLOAT_EQ(x[states::vz], 0.0);

    EXPECT_FLOAT_EQ(x[states::qx], 0.0);
    EXPECT_FLOAT_EQ(x[states::qy], 0.0);
    EXPECT_FLOAT_EQ(x[states::qz], 1.0);
    EXPECT_NEAR(x[states::qw], 0.0, 1e-7);
}

TEST(ConstantVelocity, Jacobian) {
    KF system_state(1e-3);
    initializeObserver(system_state);

    auto J = MotionModel::systemJacobian(system_state);
    //~ IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
    //~ std::cout << std::endl << J.format(CommaInitFmt) << std::endl;
    Matrix<Scalar, StateDescription::number_of_states, StateDescription::number_of_states> Jref;
    Jref << 1, 0, 0, 0.001, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 1, 0, 0, 0.001, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 1, 0, 0, 0.001, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 1, 0, 0, 0, -0.0005, 0, 0,  0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, -0.0005, 0,  0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, -0.0005,  0, 0, 0, 0, 0, 0, 0, 0, 0, 1.00002, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
    EXPECT_FLOAT_EQ((J-Jref).maxCoeff(), 0);
}
