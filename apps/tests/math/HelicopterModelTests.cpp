#include <gtest/gtest.h>

#include <sys/math/models.hpp>
#include <os/mem/ProtectedData.hpp>
#include <sys/math/filtering/EKF.hpp>
#include <cmath>
#include <Eigen/Core>
#include <os/utils/params.hpp>

using namespace sys;
using namespace Eigen;

typedef math::models::HelicopterStates States;
typedef math::GaussianFilter<States> Filter;
typedef math::models::HelicopterMotion<States> MotionModel;
typedef math::EKF Algorithm;

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    os::initParameters();
    return RUN_ALL_TESTS();
}

class HelicopterModelTests : public ::testing::Test {
    public:
        Filter system_state;
        HelicopterModelTests() {
            math::models::helicopter::Parameters<>::initializeFromParams();
            States::initialize(system_state);
        }
};

TEST_F(HelicopterModelTests, SingleDimension) {
    system_state.state[States::vx] = 1.0;
    EXPECT_FLOAT_EQ(system_state.state[States::x], 0.0);
    EXPECT_FLOAT_EQ(system_state.state[States::qw], 1.0);

    auto x = MotionModel::predict<States>(system_state.state, 1.0);
    std::cout << x.transpose() << std::endl;
    EXPECT_FLOAT_EQ(x[States::x], 1.0);
    EXPECT_FLOAT_EQ(x[States::vx], 1.0);
    EXPECT_FLOAT_EQ(x[States::y], 0.0);
    EXPECT_FLOAT_EQ(x[States::vy], 0.0);
}
#if 0
TEST_F(HelicopterModelTests, Diagonal) {
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

TEST_F(HelicopterModelTests, RotateX) {
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

TEST_F(HelicopterModelTests, RotateY) {
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

TEST_F(HelicopterModelTests, RotateZ) {
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

TEST_F(HelicopterModelTests, Jacobian) {
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

#endif
