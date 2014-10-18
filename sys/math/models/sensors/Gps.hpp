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
                typedef SCart3D States;
                typedef States::StateVector Result;
                static const int nofStates = States::nofStates;

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

                template<typename DerivationStates, typename ExternalStates>
                static Matrix<Scalar, nofStates, DerivationStates::nofStates>
                observationMatrix(const typename ExternalStates::StateVector&) {
                    typedef Matrix<Scalar, nofStates, DerivationStates::nofStates> JacobianMatrix;
                    JacobianMatrix J;
                    J.setZero();

                    J(States::x, DerivationStates::x) = 1;
                    J(States::y, DerivationStates::y) = 1;
                    J(States::z, DerivationStates::z) = 1;

                    /*
                     *J(States::vx, States::vx) = 1;
                     *J(States::vy, States::vy) = 1;
                     *J(States::vz, States::vz) = 1;
                     */

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
