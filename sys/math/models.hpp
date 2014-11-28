#pragma once
#ifndef SYS_MATH_MODELS_HPP_
#define SYS_MATH_MODELS_HPP_

#include <Eigen/Core>
#include <sys/types.hpp>

namespace sys {
    namespace math {
        namespace models {
            using namespace Eigen;
            template<typename Parent_, typename States_, bool ISDISCRETE=true>
            struct Model {
                typedef Parent_ Parent;
                typedef States_ States;
                using StateVector = typename States::StateVector;
                static const int nofStates = States::nofStates;
                static const bool isDiscrete = ISDISCRETE;

                static Matrix<Scalar, nofStates, nofStates>
                covariance(const Scalar dT) {
                    typedef Matrix<Scalar, nofStates, nofStates> RetType;
                    return RetType::Identity() * dT;
                }

                template<typename ExternalStates>
                static void update(typename ExternalStates::StateVector& x, const Scalar dT) {
                    StateVector r = Parent::template predict<ExternalStates>(x, dT);
                    for(int i=0; i < nofStates; ++i) {
                        x(States::template statemap<ExternalStates>(i)) = r(i);
                    }
                }
            };
        }
    }
}

#include <sys/math/models/motion.hpp>
#include <sys/math/models/sensors.hpp>

#endif
