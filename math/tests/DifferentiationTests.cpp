#include <gtest/gtest.h>

#include <sys/math/differentiation.hpp>
#include <sys/types.hpp>
#include <Eigen/Core>

using namespace Eigen;
using namespace sys;

typedef Matrix<Scalar, 2, 1> Vec;
typedef Matrix<Scalar, 2, 2> Mat;

Vec singleDimConstantVelocity(const Vec& x, const Scalar& dT) {
    Vec ret = x + Vec(x(1)*dT, 0);
    //~ std::cout << "In: " << x.transpose() << std::endl << "Out: " << ret.transpose() << std::endl;
    return ret;
}

TEST(Differentation, SingleDimensionConstantVelocity) {
    Vec dx = Vec::Constant(1e-3);
    Vec x(0.0, 1.0);
    Scalar dT = 1e-3;
    Mat diff = math::differentiate<Vec, Scalar, singleDimConstantVelocity>(x, dx, dT);
    EXPECT_NEAR(diff(0,0), 1.0, 1e-5); EXPECT_NEAR(diff(0,1), dT, 1e-5);
    EXPECT_NEAR(diff(1,0), 0.0, 1e-5); EXPECT_NEAR(diff(1,1), 1.0, 2e-5);
}
