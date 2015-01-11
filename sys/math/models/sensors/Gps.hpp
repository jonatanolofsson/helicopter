#pragma once
#ifndef SYS_MATH_MODELS_SENSORS_GPS_HPP_
#define SYS_MATH_MODELS_SENSORS_GPS_HPP_

#include <sys/math/constants.hpp>
#include <sys/math/filtering/GaussianFilter.hpp>
#include <Eigen/Core>
#include <os/com/Dispatcher.hpp>
#include <sys/types.hpp>
#include <sys/math/statistics.hpp>

namespace sys {
    namespace math {
        namespace models {
            using namespace Eigen;
            struct Gps {
                typedef Gps Self;
                typedef X_3D States;
                typedef States::StateVector Result;
                static const int nofStates = States::nofStates;
                static const int frequency = 10;

                template<typename ExternalStates>
                static Result measurement(const typename ExternalStates::StateVector& state) {
                    typedef ExternalStates extstates;
                    Result m;
                    m[States::x] = state[extstates::x];
                    m[States::y] = state[extstates::y];
                    m[States::z] = state[extstates::z];

                    /*
                     *m[States::vx] = state[extstates::vx];
                     *m[States::vy] = state[extstates::vy];
                     *m[States::vz] = state[extstates::vz];
                     */

                    return m;
                }

                static internal::Covariance<nofStates> covariance() {
                    typedef Matrix<Scalar, nofStates, nofStates> RetType;
                    return RetType::Identity();
                }

                template<typename ExternalStates>
                static Matrix<Scalar, nofStates, ExternalStates::nofStates>
                observationMatrix(const typename ExternalStates::StateVector&) {
                    typedef Matrix<Scalar, nofStates, ExternalStates::nofStates> JacobianMatrix;
                    JacobianMatrix J;
                    J.setZero();

                    J(States::x, ExternalStates::x) = 1;
                    J(States::y, ExternalStates::y) = 1;
                    J(States::z, ExternalStates::z) = 1;

                    return J;
                }

                static Result noise() {
                    return math::normalSample(covariance());
                }
            };
        }
    }
}

#endif
