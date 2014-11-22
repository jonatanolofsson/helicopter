#include <gtest/gtest.h>

#include <sys/math/models.hpp>
#include <os/mem/ProtectedData.hpp>
#include <sys/math/filtering/EKF.hpp>
#include <cmath>
#include <Eigen/Core>

using namespace sys;
using namespace Eigen;

typedef math::models::VWXQ_3D States;
typedef math::GaussianFilter<States> Filter;
typedef math::models::Velocity_XQ_3D<States> MotionModel;
typedef math::EKF Algorithm;

void initializeObserver(Filter& system_state) {
    States::initialize(system_state);
}

TEST(ConstantVelocity, SingleDimension) {
    Filter system_state;
    initializeObserver(system_state);
    system_state.state[States::vx] = 1.0;
    EXPECT_FLOAT_EQ(system_state.state[States::x], 0.0);
    EXPECT_FLOAT_EQ(system_state.state[States::qw], 1.0);

    auto x = MotionModel::predict<States>(system_state.state, 1.0);
    EXPECT_FLOAT_EQ(x[States::x], 1.0);
    EXPECT_FLOAT_EQ(x[States::vx], 1.0);
    EXPECT_FLOAT_EQ(x[States::y], 0.0);
    EXPECT_FLOAT_EQ(x[States::vy], 0.0);
}

TEST(ConstantVelocity, Diagonal) {
    Filter system_state;
    initializeObserver(system_state);
    system_state.state[States::vx] = 1.0;
    system_state.state[States::vy] = 1.0;
    system_state.state[States::vz] = 1.0;
    EXPECT_FLOAT_EQ(system_state.state[States::x], 0.0);
    EXPECT_FLOAT_EQ(system_state.state[States::qw], 1.0);

    auto x = MotionModel::predict<States>(system_state.state, 1.0);
    EXPECT_FLOAT_EQ(x[States::x], 1.0);
    EXPECT_FLOAT_EQ(x[States::vx], 1.0);
    EXPECT_FLOAT_EQ(x[States::y], 1.0);
    EXPECT_FLOAT_EQ(x[States::vy], 1.0);
    EXPECT_FLOAT_EQ(x[States::z], 1.0);
    EXPECT_FLOAT_EQ(x[States::vz], 1.0);

    EXPECT_FLOAT_EQ(x[States::qx], 0.0);
    EXPECT_FLOAT_EQ(x[States::qy], 0.0);
    EXPECT_FLOAT_EQ(x[States::qz], 0.0);
    EXPECT_FLOAT_EQ(x[States::qw], 1.0);
}

TEST(ConstantVelocity, RotateX) {
    Filter system_state;
    initializeObserver(system_state);
    system_state.state[States::wx] = 1.0;

    auto x = MotionModel::predict<States>(system_state.state, M_PI);
    EXPECT_FLOAT_EQ(x[States::x], 0.0);
    EXPECT_FLOAT_EQ(x[States::vx], 0.0);
    EXPECT_FLOAT_EQ(x[States::y], 0.0);
    EXPECT_FLOAT_EQ(x[States::vy], 0.0);
    EXPECT_FLOAT_EQ(x[States::z], 0.0);
    EXPECT_FLOAT_EQ(x[States::vz], 0.0);

    EXPECT_FLOAT_EQ(x[States::qx], -1.0);
    EXPECT_FLOAT_EQ(x[States::qy], 0.0);
    EXPECT_FLOAT_EQ(x[States::qz], 0.0);
    EXPECT_NEAR(x[States::qw], 0.0, 1e-7);
}

TEST(ConstantVelocity, RotateY) {
    Filter system_state;
    initializeObserver(system_state);
    system_state.state[States::wy] = 1.0;

    auto x = MotionModel::predict<States>(system_state.state, M_PI);
    EXPECT_FLOAT_EQ(x[States::x], 0.0);
    EXPECT_FLOAT_EQ(x[States::vx], 0.0);
    EXPECT_FLOAT_EQ(x[States::y], 0.0);
    EXPECT_FLOAT_EQ(x[States::vy], 0.0);
    EXPECT_FLOAT_EQ(x[States::z], 0.0);
    EXPECT_FLOAT_EQ(x[States::vz], 0.0);

    EXPECT_FLOAT_EQ(x[States::qx], 0.0);
    EXPECT_FLOAT_EQ(x[States::qy], -1.0);
    EXPECT_FLOAT_EQ(x[States::qz], 0.0);
    EXPECT_NEAR(x[States::qw], 0.0, 1e-7);
}

TEST(ConstantVelocity, RotateZ) {
    Filter system_state;
    initializeObserver(system_state);
    system_state.state[States::wz] = -1.0;

    auto x = MotionModel::predict<States>(system_state.state, M_PI);
    EXPECT_FLOAT_EQ(x[States::x], 0.0);
    EXPECT_FLOAT_EQ(x[States::vx], 0.0);
    EXPECT_FLOAT_EQ(x[States::y], 0.0);
    EXPECT_FLOAT_EQ(x[States::vy], 0.0);
    EXPECT_FLOAT_EQ(x[States::z], 0.0);
    EXPECT_FLOAT_EQ(x[States::vz], 0.0);

    EXPECT_FLOAT_EQ(x[States::qx], 0.0);
    EXPECT_FLOAT_EQ(x[States::qy], 0.0);
    EXPECT_FLOAT_EQ(x[States::qz], 1.0);
    EXPECT_NEAR(x[States::qw], 0.0, 1e-7);
}

TEST(ConstantVelocity, Jacobian) {
    Filter system_state;
    initializeObserver(system_state);

    auto static const dT = settings::dT;
    auto J = sys::math::differentiate<MotionModel, States, States>(system_state.state);
    //~ IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
    //~ std::cout << std::endl << J.format(CommaInitFmt) << std::endl;
    Matrix<Scalar, States::nofStates, States::nofStates> Jref;
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
