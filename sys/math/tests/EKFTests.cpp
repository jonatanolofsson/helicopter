#include <gtest/gtest.h>

#include <sys/math/models.hpp>
#include <sys/math/filtering.hpp>
#include <sys/math/States.hpp>
#include <Eigen/Core>


using namespace Eigen;
using namespace sys;

typedef math::models::SCart3DQuat<> States;
typedef math::GaussianFilter<States> Filter;
typedef math::models::ConstantVelocities6D MotionModel;
typedef math::EKF Algorithm;


class EKFTests : public ::testing::Test {
    public:
        Filter filter;
        EKFTests()
        {
            States::initialize(filter);
        }
};

TEST_F(EKFTests, TimeUpdateStandStill) {
    auto reference = filter.state;
    Algorithm::timeUpdate<MotionModel>(filter, 1.0);
    EXPECT_TRUE((reference-filter.state).norm() < 1e-3) << reference.transpose() << " = " << filter.state.transpose() << "  [ " << (reference-filter.state).transpose() << " ]";
}

TEST_F(EKFTests, TimeUpdateMovingPositiveX) {
    filter.state[States::vx] = 1.0;
    auto reference = filter.state;
    reference[States::x] = 1.0;

    Algorithm::timeUpdate<MotionModel>(filter, 1.0);
    EXPECT_TRUE((reference-filter.state).norm() < 1e-3) << reference.transpose() << " = " << filter.state.transpose() << "  [ " << (reference-filter.state).transpose() << " ]";
}

TEST_F(EKFTests, TimeUpdateMovingNegativeX) {
    filter.state[States::vx] = -1.0;
    auto reference = filter.state;
    reference[States::x] = -1.0;

    Algorithm::timeUpdate<MotionModel>(filter, 1.0);
    EXPECT_TRUE((reference-filter.state).norm() < 1e-3) << reference.transpose() << " = " << filter.state.transpose() << "  [ " << (reference-filter.state).transpose() << " ]";
}

TEST_F(EKFTests, TimeUpdateRotationPositiveX) {
    filter.state[States::wx] = 1.0;
    auto reference = filter.state;
    reference[States::qw] = 0;
    reference[States::qx] = -1.0;

    Algorithm::timeUpdate<MotionModel>(filter, M_PI);
    EXPECT_TRUE((reference-filter.state).norm() < 1e-7) << reference.transpose() << " = " << filter.state.transpose();
}

TEST_F(EKFTests, TimeUpdateRepeatedRotationPositiveX) {
    filter.state[States::wx] = 1.0;
    auto reference = filter.state;
    reference[States::qw] = 0;
    reference[States::qx] = -1.0;

    for(int i = 0; i < 100; ++i) {
        Algorithm::timeUpdate<MotionModel>(filter, M_PI/100);
    }
    EXPECT_TRUE((reference-filter.state).norm() < 1e-5) << reference.transpose() << " = " << filter.state.transpose();
}


TEST_F(EKFTests, MeasurementUpdateGPS) {
    auto reference = filter.state;
    reference[States::x] = reference[States::y] = reference[States::z] = 0.5;

    math::GaussianMeasurement<sys::math::models::Gps<States>> m;
    m.z << 1, 1, 1;
    m.R.setIdentity();

    Algorithm::measurementUpdate<>(filter, m);
    EXPECT_TRUE((reference-filter.state).norm() < 1e-5) << reference.transpose() << " = " << filter.state.transpose();
}
