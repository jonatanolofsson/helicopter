#include <gtest/gtest.h>

#include <sys/math/models.hpp>
#include <sys/math/filtering.hpp>
#include <sys/math/states.hpp>
#include <Eigen/Core>


using namespace Eigen;
using namespace sys;
static const int nofParticles = 2000;
typedef math::models::S2DPose states;
typedef math::models::CVW controls;
typedef math::models::Description<states, controls> ModelDescription;
typedef math::models::CoordinatedTurn2DPose<ModelDescription> MotionModel;
typedef math::ParticleFilter<ModelDescription, nofParticles> Filter;
typedef math::PF FilterAlgorithm;


class PFTests : public ::testing::Test {
    public:
        Filter filter;
        ModelDescription::Controls u;
        PFTests()
        {
            states::initialize(filter);
            u.setZero();
            for(auto& p : filter.particles) {
                p.state.setZero();
                p.weight = 1.0/nofParticles;
            }
        }
};

TEST_F(PFTests, TimeUpdateStandStill) {
    auto reference = filter.state;
    FilterAlgorithm::timeUpdate<MotionModel>(filter, u, 1.0);
    std::cout << filter.state.transpose() << std::endl;
    EXPECT_EQ(reference, filter.state) << reference.transpose() << " = " << filter.state.transpose();
}

TEST_F(PFTests, TimeUpdateMoving) {
    u[controls::v] = 1.0;
    auto reference = filter.state;
    reference[states::x] = 1.0;

    FilterAlgorithm::timeUpdate<MotionModel>(filter, u, 1.0);
    EXPECT_EQ(reference, filter.state) << reference.transpose() << " = " << filter.state.transpose();
}

TEST_F(PFTests, TimeUpdateMovingNegative) {
    u[controls::v] = -1.0;
    auto reference = filter.state;
    reference[states::x] = -1.0;

    FilterAlgorithm::timeUpdate<MotionModel>(filter, u, 1.0);
    EXPECT_EQ(reference, filter.state) << reference.transpose() << " = " << filter.state.transpose();
}

TEST_F(PFTests, TimeUpdateRotationPositiveX) {
    u[controls::w] = 1.0;
    auto reference = filter.state;
    reference[states::th] = M_PI;

    FilterAlgorithm::timeUpdate<MotionModel>(filter, u, M_PI);
    EXPECT_TRUE((reference-filter.state).norm() < 1e-7) << reference.transpose() << " = " << filter.state.transpose();
}

TEST_F(PFTests, TimeUpdateRepeatedRotationPositiveX) {
    u[controls::w] = 1.0;
    auto reference = filter.state;
    reference[states::th] = M_PI;

    for(int i = 0; i < 100; ++i) {
        FilterAlgorithm::timeUpdate<MotionModel>(filter, u, M_PI/100);
    }
    EXPECT_TRUE((reference-filter.state).norm() < 1e-5) << reference.transpose() << " = " << filter.state.transpose();
}


//~ TEST_F(PFTests, MeasurementUpdateGPS) {
    //~ Filter filter;
    //~ states::initialize(filter);
    //~ auto reference = filter.state;
    //~ reference[states::x] = reference[states::y] = reference[states::z] = 0.5;
//~
    //~ math::GaussianMeasurement<sys::models::sensors::Gps> m;
    //~ m.z << 1, 1, 1, 0, 0, 0;
    //~ m.R.setIdentity();
//~
    //~ FilterAlgorithm::measurementUpdate<>(filter, m);
    //~ EXPECT_TRUE((reference-filter.state).norm() < 1e-5) << reference.transpose() << " = " << filter.state.transpose();
//~ }
