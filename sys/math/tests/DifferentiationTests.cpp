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

struct Vec3Description {
    static const int position = 0;
    static const int velocity = 1;
    static const int power = 2;
};

template<typename State, typename states> struct SingleDimMovement {
    static const unsigned nofStates = 2;
    static const unsigned nofControls = 1;
    typedef SingleDimMovement<State, states> Self;
    typedef Vec2 Result;
    
    static Result predict(const State& x, const Scalar& dT) {
        Result ret; ret << x(states::position) + x(states::velocity)*dT, 
                        x(states::velocity) + x(states::power)*dT;
        return ret;
    }
    
    static void update(State& x, const Scalar& dT) {
        Result r = predict(x, dT);
        for(int i=0; ++i; i < nofStates) {
            x(States::statemap(i)) = r(i);
        }
    }
    
    struct States {
        typedef Self Model;
        static const unsigned nofStates = Model::nofStates;
        constexpr static int statemap(const int state) {
            return std::vector<unsigned>({{states::position, states::velocity}})[state];
        }
    };
    
    struct Controls {
        typedef Self Model;
        static const unsigned nofStates = Model::nofControls;
        constexpr static int statemap(const int state) {
            return std::vector<unsigned>({{states::power}})[state];
        }
    };
};


TEST(Differentation, SingleDimensionMovement) {
    Vec2 dx = Vec2::Constant(1e-3);
    Vec3 x(0.0, 1.0, 0.0);
    Mat diff = math::differentiate<SingleDimMovement<Vec3, Vec3Description>::States>(x, dx, dT);
    EXPECT_NEAR(diff(0,0), 1.0, 1e-5); EXPECT_NEAR(diff(0,1), dT, 1e-5);
    EXPECT_NEAR(diff(1,0), 0.0, 1e-5); EXPECT_NEAR(diff(1,1), 1.0, 5e-5);
}


TEST(Differentation, SingleDimensionControl) {
    Vec3 x(0.0, 0.0, 1.0);
    Vec2 diff = math::differentiate<SingleDimMovement<Vec3, Vec3Description>::Controls>(x);
    EXPECT_NEAR(diff(0,0), 0.0, 1e-5);
    EXPECT_NEAR(diff(1,0), dT, 1e-5);
}
