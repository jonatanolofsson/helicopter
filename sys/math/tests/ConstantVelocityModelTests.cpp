#include <gtest/gtest.h>

#include <sys/math/models.hpp>
#include <os/mem/ProtectedData.hpp>
#include <sys/math/filtering/EKF.hpp>
#include <cmath>
#include <Eigen/Core>

using namespace sys;
using namespace Eigen;

typedef math::models::SCart3DQuat<> StateDescription;
typedef StateDescription states;
typedef math::GaussianFilter<StateDescription> Filter;
typedef math::models::ConstantVelocities6D MotionModel;
typedef math::EKF Algorithm;

void initializeObserver(Filter& system_state) {
    StateDescription::initialize(system_state);
}

TEST(ConstantVelocity, SingleDimension) {
    Filter system_state;
    initializeObserver(system_state);
    system_state.state[states::vx] = 1.0;
    EXPECT_FLOAT_EQ(system_state.state[states::x], 0.0);
    EXPECT_FLOAT_EQ(system_state.state[states::qw], 1.0);

    auto x = MotionModel::predict<StateDescription>(system_state.state, 1.0);
    EXPECT_FLOAT_EQ(x[states::x], 1.0);
    EXPECT_FLOAT_EQ(x[states::vx], 1.0);
    EXPECT_FLOAT_EQ(x[states::y], 0.0);
    EXPECT_FLOAT_EQ(x[states::vy], 0.0);
}

TEST(ConstantVelocity, Diagonal) {
    Filter system_state;
    initializeObserver(system_state);
    system_state.state[states::vx] = 1.0;
    system_state.state[states::vy] = 1.0;
    system_state.state[states::vz] = 1.0;
    EXPECT_FLOAT_EQ(system_state.state[states::x], 0.0);
    EXPECT_FLOAT_EQ(system_state.state[states::qw], 1.0);

    auto x = MotionModel::predict<StateDescription>(system_state.state, 1.0);
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
    Filter system_state;
    initializeObserver(system_state);
    system_state.state[states::wx] = 1.0;

    auto x = MotionModel::predict<StateDescription>(system_state.state, M_PI);
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
    Filter system_state;
    initializeObserver(system_state);
    system_state.state[states::wy] = 1.0;

    auto x = MotionModel::predict<StateDescription>(system_state.state, M_PI);
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
    Filter system_state;
    initializeObserver(system_state);
    system_state.state[states::wz] = -1.0;

    auto x = MotionModel::predict<StateDescription>(system_state.state, M_PI);
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
    Filter system_state;
    initializeObserver(system_state);

    auto static const dT = settings::dT;
    auto J = MotionModel::systemJacobian<StateDescription>(system_state.state);
    //~ IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
    //~ std::cout << std::endl << J.format(CommaInitFmt) << std::endl;
    Matrix<Scalar, StateDescription::nofStates, StateDescription::nofStates> Jref;
    Jref.setZero();
    Jref << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
            dT, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
            0, dT, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, dT, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, -0.005, 0, 0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, -0.005, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, -0.005, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.00003;
    EXPECT_LT((J-Jref).maxCoeff(), 1e-5) << J-Jref;
}
