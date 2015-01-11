#pragma once
#ifndef SYS_MATH_MODELS_CT_XYT_1D_HPP_
#define SYS_MATH_MODELS_CT_XYT_1D_HPP_

#include <Eigen/Core>
#include <sys/types.hpp>
#include <sys/math/constants.hpp>
#include <sys/math/algorithm.hpp>
#include <cmath>

namespace sys {
    namespace math {
        namespace models {
            using namespace Eigen;
            template<typename States_>
            struct CT_XYT_1D : public Model<CT_XYT_1D<States_>, States_> {
                typedef CT_XYT_1D<States_> Self;
                typedef Model<Self, States_> Base;
                using States = typename Base::States;
                using StateVector = typename Base::StateVector;

                template<typename ExternalStates>
                static StateVector predict(const typename ExternalStates::StateVector& x, const Scalar dT) {
                    typedef ExternalStates extstates;
                    StateVector xnext = States::template translateFrom<ExternalStates>(x);
                    using std::sin; using std::cos;

                    Scalar a;
                    if(x(extstates::w) < 1e-2) {
                        a = x(extstates::v) * dT;
                    } else {
                        a = 2. * x(extstates::v) * sin(x(extstates::w) * dT / 2.) / x(extstates::w);
                    }
                    const auto b = x(extstates::th) + (x(extstates::w) * dT / 2.);

                    xnext(States::x) += a*cos(b);
                    xnext(States::y) += a*sin(b);
                    xnext(States::th) += x(extstates::w)*dT;
                    return xnext;
                }
            };
        }
    }
}

#endif
