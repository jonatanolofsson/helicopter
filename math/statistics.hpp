#pragma once
#ifndef SYS_MATH_STATISTICS_HPP_
#define SYS_MATH_STATISTICS_HPP_
#include <Eigen/Dense>
#include <functional>
#include <random>
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
        template<typename Scalar = Scalar>
        Scalar randN() {
            static std::default_random_engine generator;
            static std::normal_distribution<Scalar> distribution(0.0, 1.0);
            return distribution(generator);
        }

        template<int N>
        int irandU() {
            static std::default_random_engine generator;
            static std::uniform_int_distribution<> distribution(0, N-1);
            return distribution(generator);
        }

        template<typename Scalar = Scalar>
        int randU() {
            static std::default_random_engine generator;
            static std::uniform_real_distribution<Scalar> distribution(0.0, 1.0);
            return distribution(generator);
        }

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


        template<typename D0, typename D1, typename D2>
        Scalar normalProbabilityUnscaled(const D0& m, const D1& z, const D2& variance) {
            Scalar p = 1.0;

            for(unsigned i = 0; i < D0::RowsAtCompileTime; ++i) {
                p *= std::exp(-SQUARE(m(i) - z(i)) * 0.5 / variance(i));
            }

            return p;
        }
    }
}

#endif
