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

template<typename M = void>
struct ControlDescription {
    typedef M Model;

    static const int power = 0;
    static const int nofStates = 1;

    template<typename ExternalStateDescription>
    constexpr static int statemap(const int state) {
        return std::vector<unsigned>{
            ExternalStateDescription::power}[state];
    }
};

template<typename M = void>
struct Vec2Description {
    typedef M Model;
    static const int position = 0;
    static const int velocity = 1;

    typedef Vec2 StateVector;

    static const unsigned nofStates = 2;
    template<typename ExternalStateDescription>
    constexpr static int statemap(const int state) {
        return std::vector<unsigned>{
            ExternalStateDescription::position,
            ExternalStateDescription::velocity}[state];
    }
};

template<typename M = void>
struct Vec3Description {
    typedef M Model;
    static const int position = 0;
    static const int velocity = 1;
    static const int power = 2;

    typedef Vec3 StateVector;
};

struct SingleDimMovement {
    typedef SingleDimMovement Self;
    typedef Vec2Description<Self> States;
    typedef States::StateVector Result;
    typedef ControlDescription<Self> Controls;

    template<typename ExternalStateDescription>
    static Result predict(const ExternalStateDescription::StateVector& x, const Scalar& dT) {
        typedef ExternalStateDescription extstates;
        Result ret; ret << x(extstates::position) + x(extstates::velocity)*dT,
                        x(extstates::velocity) + x(extstates::power)*dT;
        return ret;
    }

    template<typename ExternalStateDescription>
    static void update(ExternalStateDescription::StateVector& x, const Scalar& dT) {
        Result r = predict<ExternalStateDescription>(x, dT);
        for(int i=0; i < States::nofStates; ++i) {
            x(States::statemap<ExternalStateDescription>(i)) = r(i);
        }
    }
};


TEST(Differentation, SingleDimensionMovement) {
    Vec2 dx = Vec2::Constant(1e-3);
    Vec3 x(0.0, 1.0, 0.0);
    Mat diff = math::differentiate<SingleDimMovement::States, Vec3Description>(x, dx, dT);
    EXPECT_NEAR(diff(0,0), 1.0, 1e-5); EXPECT_NEAR(diff(0,1), dT, 1e-5);
    EXPECT_NEAR(diff(1,0), 0.0, 1e-5); EXPECT_NEAR(diff(1,1), 1.0, 5e-5);
}


TEST(Differentation, SingleDimensionControl) {
    Vec3 x(0.0, 0.0, 1.0);
    Vec2 diff = math::differentiate<SingleDimMovement::Controls, Vec3Description>(x);
    EXPECT_NEAR(diff(0,0), 0.0, 1e-5);
    EXPECT_NEAR(diff(1,0), dT, 1e-5);
>}
