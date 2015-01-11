#pragma once
#ifndef SYS_MATH_MODELS_VELOCITY_X_3D_HPP_
#define SYS_MATH_MODELS_VELOCITY_X_3D_HPP_

#include <Eigen/Core>
#include <sys/types.hpp>
#include <sys/math/states.hpp>
#include <sys/math/constants.hpp>
#include <sys/math/algorithm.hpp>
#include <cmath>

#include <iostream>

namespace sys {
    namespace math {
        namespace models {
            using namespace Eigen;
            template<typename States_ = X_3D>
            struct Velocity_X_3D : public Model<Velocity_X_3D<States_>, States_> {
                typedef Velocity_X_3D<States_> Self;
                typedef Model<Self, States_> Base;
                using States = typename Base::States;
                using StateVector = typename Base::StateVector;

                template<typename ExternalStates>
                static StateVector predict(const typename ExternalStates::StateVector& x, const Scalar dT) {
                    typedef ExternalStates extstates;
                    StateVector xnext = States::template translateFrom<ExternalStates>(x);

                    xnext.template segment<3>(States::position) += x.template segment<3>(extstates::velocity) * dT;
                    return xnext;
                }
            };
        }
    }
}

#endif
