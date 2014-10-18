#include <gtest/gtest.h>

#include <sys/math/algorithm/differentiation.hpp>
#include <sys/types.hpp>
#include <Eigen/Core>
#include <vector>

using namespace Eigen;
using namespace sys;

typedef Matrix<Scalar, 2, 1> Vec1;
typedef Matrix<Scalar, 2, 1> Vec2;
typedef Matrix<Scalar, 3, 1> Vec3;
typedef Matrix<Scalar, 2, 2> Mat;
static const Scalar dT = 1e-2;

struct ControlDescription {
    static const int power = 0;
    static const int nofStates = 1;

    typedef Vec1 StateVector;

    template<typename ExternalStates>
    constexpr static int statemap(const int state) {
        return std::vector<int>{
            ExternalStates::power}[state];
    }
};

struct Vec2Description {
    static const int position = 0;
    static const int velocity = 1;
    static const int nofStates = 2;

    typedef Vec2 StateVector;

    template<typename ExternalStates>
    constexpr static int statemap(const int state) {
        return std::vector<int>{
            ExternalStates::position,
            ExternalStates::velocity}[state];
    }
};

struct Vec3Description {
    static const int position = 0;
    static const int velocity = 1;
    static const int power = 2;

    typedef Vec3 StateVector;

    template<typename ExternalStates>
    constexpr static int statemap(const int state) {
        return std::vector<int>{
            ExternalStates::position,
            ExternalStates::velocity,
            ExternalStates::power}[state];
    }
};

struct SingleDimMovement {
    typedef SingleDimMovement Self;
    typedef Vec2Description States;
    typedef States::StateVector Result;
    static const int nofStates = States::nofStates;

    template<typename ExternalStates>
    static Result predict(const typename ExternalStates::StateVector& x, const Scalar& dT) {
        typedef ExternalStates extstates;
        Result ret; ret << (x(extstates::position) + x(extstates::velocity)*dT),
                           x(extstates::velocity) + x(extstates::power)*dT;
        return ret;
    }

    template<typename ExternalStates>
    static void update(typename ExternalStates::StateVector& x, const Scalar& dT) {
        Result r = predict<ExternalStates>(x, dT);
        for(int i=0; i < States::nofStates; ++i) {
            x(States::statemap<ExternalStates>(i)) = r(i);
        }
    }
};


TEST(Differentation, SingleDimensionMovement) {
    Vec2 dx = Vec2::Constant(1e-3);
    Vec3 x(0.0, 1.0, 0.0);
    Mat diff = math::differentiate<SingleDimMovement, Vec2Description, Vec3Description>(x, dx, dT);
    EXPECT_NEAR(diff(0,0), 1.0, 1e-5); EXPECT_NEAR(diff(0,1), dT, 1e-5);
    EXPECT_NEAR(diff(1,0), 0.0, 1e-5); EXPECT_NEAR(diff(1,1), 1.0, 5e-5);
}


TEST(Differentation, SingleDimensionControl) {
    Vec3 x(0.0, 0.0, 1.0);
    Vec2 diff = math::differentiate<SingleDimMovement, ControlDescription, Vec3Description>(x);
    EXPECT_NEAR(diff(0,0), 0.0, 1e-5);
    EXPECT_NEAR(diff(1,0), dT, 1e-5);
}
