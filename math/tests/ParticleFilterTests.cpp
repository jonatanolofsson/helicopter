#include <gtest/gtest.h>

#include <cmath>
#include <sys/math/models.hpp>
#include <sys/math/models/Map.hpp>
#include <sys/math/filtering.hpp>
#include <sys/math/base.hpp>
#include <sys/math/states.hpp>
#include <sys/math/statistics.hpp>
#include <Eigen/Core>


using namespace Eigen;
using namespace sys;
static const int nofParticles = 5000;
typedef math::models::S2DPose states;
typedef math::models::CVW controls;
typedef math::models::Description<states, controls> ModelDescription;
typedef math::models::CoordinatedTurn2DPose<ModelDescription> MotionModel;
typedef math::ParticleFilter<ModelDescription, nofParticles> Filter;
typedef math::PF Algorithm;


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
    Algorithm::timeUpdate<MotionModel>(filter, u, 1.0);

    EXPECT_LT((reference-filter.state).norm(), 1e-2) << reference.transpose() << " = " << filter.state.transpose();
}

TEST_F(PFTests, TimeUpdateMoving) {
    u[controls::v] = 1.0;
    auto reference = filter.state;
    reference[states::x] = 1.0;

    Algorithm::timeUpdate<MotionModel>(filter, u, 1.0);

    EXPECT_LT((reference-filter.state).norm(), 1e-2) << reference.transpose() << " = " << filter.state.transpose();
}

TEST_F(PFTests, TimeUpdateMovingNegative) {
    u[controls::v] = -1.0;
    auto reference = filter.state;
    reference[states::x] = -1.0;

    Algorithm::timeUpdate<MotionModel>(filter, u, 1.0);

    EXPECT_LT((reference-filter.state).norm(), 1e-2) << reference.transpose() << " = " << filter.state.transpose();
}

TEST_F(PFTests, TimeUpdateRotationPositiveX) {
    u[controls::w] = 1.0;
    auto reference = filter.state;
    reference[states::th] = M_PI;

    Algorithm::timeUpdate<MotionModel>(filter, u, M_PI);
    EXPECT_LT((reference-filter.state).norm(), 1e-2) << reference.transpose() << " = " << filter.state.transpose();
}


TEST_F(PFTests, WeightUpdate) {
    typedef sys::math::models::UltrasoundInMap<ModelDescription, 4, math::Map<14>> Sensor;

    u[controls::w] = 5.0;

    Sensor sensor{{
        {Sensor::SensorType::Position(0.1, 0), 0},
        {Sensor::SensorType::Position(0.0, 0.1), M_PI_2},
        {Sensor::SensorType::Position(-0.1, 0), M_PI},
        {Sensor::SensorType::Position(0.0, -0.1), -M_PI_2}
    },
    {{
        // border
        {{0.00, 0.00}, {0.00, 2.40}},
        {{0.00, 2.40}, {2.40, 2.40}},
        {{2.40, 2.40}, {2.40, 0.00}},
        {{2.40, 0.00}, {0.00, 0.00}},

        // vertical walls
        {{0.69, 1.51}, {0.69, 2.40}},
        {{0.69, 0.46}, {0.69, 1.01}},
        {{1.17, 0.00}, {1.17, 0.46}},
        {{1.17, 1.37}, {1.17, 1.94}},
        {{1.94, 1.37}, {1.94, 1.94}},

        // horizontal walls
        {{1.17, 1.94}, {1.94, 1.94}},
        {{0.46, 1.51}, {0.69, 1.51}},
        {{1.65, 1.37}, {1.94, 1.37}},
        {{0.00, 1.01}, {0.69, 1.01}},
        {{1.17, 0.89}, {2.40, 0.89}}
    }}};

    filter.state << 0.4, 0.7, 0.0;
    ModelDescription::States trueState = filter.state;

    for(auto& p : filter.particles) {
        p.state << math::randU()*2.2 + 0.1, math::randU()*2.2 + 0.1, math::randU()*2*M_PI;
        p.weight = 1.0/nofParticles;
    }
    filter.maxWeight = 1.0/nofParticles;

    math::GaussianMeasurement<Sensor, true> m;
    m.sensor = &sensor;
    m.z << 0.19, 0.20, 0.30, 0.60;
    m.R.setIdentity();
    m.R *= 0.1;

    auto rz = m.measurement(filter.state);
    EXPECT_LT((m.z-rz).norm(), 1e-2) << m.z.transpose() << " = " << rz.transpose();

    for(int i = 0; i < 400; ++i) {
        trueState = MotionModel::predict(trueState, u, 1e-2);
        m.z = m.measurement(trueState);
        Algorithm::update<MotionModel>(filter, u, m, 1e-2);
    }
    filter.state(states::th) = sys::math::toPi(filter.state(states::th));
    trueState(states::th) = sys::math::toPi(trueState(states::th));
    EXPECT_LT((trueState-filter.state).norm(), 1e-1) << trueState.transpose() << " = " << filter.state.transpose();
}
