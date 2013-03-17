#ifndef SYS_MATH_STATISTICS_HPP_
#define SYS_MATH_STATISTICS_HPP_
#include <Eigen/Dense>
#include <functional>
#include <math.h>

#include <sys/types.hpp>

namespace sys {
    namespace math {
        template<typename Scalar> class RandN;
    }
}

namespace Eigen
{
    namespace internal {
        template<typename Scalar> struct functor_traits<sys::math::RandN<Scalar> >    { enum { Cost = 1000, PacketAccess = false, IsRepeatable = false }; };
    }
}

namespace sys {
    namespace math {
        using namespace Eigen;
        template<typename T> T randN();

        template<typename Scalar>
        struct RandN {
            template<typename Index>
            inline const Scalar operator()(const Index, const Index = 0) const { return randN<Scalar>(); }
        };

        template<typename Derived>
        Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, 1> normalSample(const Derived& x) {
            static RandN<typename Derived::Scalar> rgen;
            SelfAdjointEigenSolver<Derived> eigenSolver(x);
            return eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal() * Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, 1>::NullaryExpr(rgen);
        }
    }
}

#endif
