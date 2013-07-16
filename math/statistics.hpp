#pragma once
#ifndef SYS_MATH_STATISTICS_HPP_
#define SYS_MATH_STATISTICS_HPP_
#include <Eigen/Dense>
#include <functional>
#include <random>
#include <math.h>

#include <sys/types.hpp>
#include <sys/math/base.hpp>

namespace sys {
    namespace math {
        template<typename S = Scalar> struct RandN;
    }
}

namespace Eigen
{
    namespace internal {
        template<typename S> struct functor_traits<sys::math::RandN<S> >    { enum { Cost = 1000, PacketAccess = false, IsRepeatable = false }; };
    }
}

namespace sys {
    namespace math {
        using namespace Eigen;
        template<typename S = Scalar>
        S randN() {
            static std::random_device rd;
            static std::default_random_engine generator(rd());
            static std::normal_distribution<S> distribution(0.0, 1.0);
            return distribution(generator);
        }

        template<int N>
        int irandU() {
            static std::random_device rd;
            static std::default_random_engine generator(rd());
            static std::uniform_int_distribution<> distribution(0, N-1);
            return distribution(generator);
        }

        template<typename S = Scalar>
        S randU() {
            static std::default_random_engine generator;
            static std::uniform_real_distribution<S> distribution(0.0, 1.0);
            return distribution(generator);
        }

        template<typename S>
        struct RandN {
            template<typename Index>
            inline const S operator()(const Index, const Index = 0) const { return randN<S>(); }
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
                //~ std::cout << "Var(i): " << variance(i,i) << std::endl;
                auto pi = std::exp(-SQUARE(m(i) - z(i)) * 0.5 / variance(i,i));
                //~ std::cout << "   m(i): " << m(i) << std::endl;
                //~ std::cout << "   z(i): " << z(i) << std::endl;
                //~ std::cout << "   m-z: " << m(i) - z(i) << std::endl;
                //~ std::cout << "   sqr(i): " << SQUARE(m(i) - z(i)) << std::endl;
                //~ std::cout << "   p(i): " << pi << std::endl;
                p *= pi;
            }

            return p;
        }
    }
}

#endif
