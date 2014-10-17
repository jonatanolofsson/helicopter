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
                typedef SCart3D<Scalar> States;
                typedef States::StateVector Result;
                typedef nofMeasurements = States::nofStates;

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

                static Matrix<Scalar, nofStates, nofStates>
                CovarianceMatrix& covariance() {
                    typedef Matrix<Scalar, nofStates, nofStates> RetType;
                    return RetType::Identity();
                }

                template<typename ExternalStates>
                static Matrix<Scalar, nofMeasurements, ExternalStates::nofStates>
                jacobian(const typename ExternalStates::StateVector&) {
                    typedef Matrix<Scalar, nofMeasurements, nofMeasurements> JacobianMatrix;
                    JacobianMatrix J;
                    J.setZero();

                    J(States::x, States::x) = 1;
                    J(States::y, States::y) = 1;
                    J(States::x, States::z) = 1;

                    /*
                     *J(States::vx, States::vx) = 1;
                     *J(States::vy, States::vy) = 1;
                     *J(States::vz, States::vz) = 1;
                     */

                    return J;
                }

                static MeasurementVector noise() {
                    return math::normalSample(covariance());
                }
            };
        }
    }
}

#endif
