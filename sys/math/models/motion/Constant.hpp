#pragma once
#ifndef SYS_MATH_MODELS_CONSTANT_HPP_
#define SYS_MATH_MODELS_CONSTANT_HPP_

#include <Eigen/Core>
#include <sys/types.hpp>
#include <sys/math/constants.hpp>
#include <sys/math/algorithm.hpp>
#include <cmath>

#include <iostream>

namespace sys {
    namespace math {
        namespace models {
            using namespace Eigen;
            template<typename States_>
            struct Constant : public Model<Constant<States_>, States_> {
                typedef Constant<States_> Self;
                typedef Model<Self, States_> Base;
                using States = typename Base::States;
                using StateVector = typename Base::StateVector;

                template<typename ExternalStates>
                static StateVector predict(const typename ExternalStates::StateVector& x, const Scalar dT __attribute__((unused))) {
                    StateVector xnext = States::template translate<ExternalStates>(x);
                    return xnext;
                }
            };
        }
    }
}

#endif
