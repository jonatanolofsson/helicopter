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
            template<typename StateDescription>
            struct Gps {
                typedef Gps<StateDescription> Self;
                typedef typename StateDescription::Scalar Scalar;
                enum state {
                    x = 0,
                    y = 1,
                    z = 2,

/*
 *                    vx = 3,
 *                    vy = 4,
 *                    vz = 5,
 *
 *                    nofMeasurements = 6
 */
                    nofMeasurements = 3
                };

                enum states {
                    position = x,
                    /*
                     *velocity = vx
                     */
                };

                typedef Matrix<Scalar, nofMeasurements, 1> MeasurementVector;
                typedef Matrix<Scalar, nofMeasurements, nofMeasurements> CovarianceMatrix;
                static MeasurementVector measurement(const typename StateDescription::StateVector& state) {
                    typedef StateDescription states;
                    MeasurementVector m;
                    m[x] = state[states::x];
                    m[y] = state[states::y];
                    m[z] = state[states::z];

                    /*
                     *m[vx] = state[states::vx];
                     *m[vy] = state[states::vy];
                     *m[vz] = state[states::vz];
                     */

                    return m;
                }

                static CovarianceMatrix cov;
                static const CovarianceMatrix& covariance() { return cov; }

                static Matrix<Scalar, nofMeasurements, StateDescription::nofStates>
                jacobian(const typename StateDescription::StateVector&) {
                    typedef StateDescription states;
                    typedef Matrix<Scalar, nofMeasurements, StateDescription::nofStates> JacobianMatrix;
                    JacobianMatrix J;
                    J.setZero();

                    J(x, states::x) = 1;
                    J(y, states::y) = 1;
                    J(z, states::z) = 1;

                    /*
                     *J(vx, states::vx) = 1;
                     *J(vy, states::vy) = 1;
                     *J(vz, states::vz) = 1;
                     */

                    return J;
                }

                static MeasurementVector noise() {
                    return math::normalSample(covariance());
                }
            };

            template<typename StateDescription> typename Gps<StateDescription>::CovarianceMatrix
            Gps<StateDescription>::cov = (Gps<StateDescription>::MeasurementVector() <<
                1.0, 1.0, 1.0//, 1.0, 1.0, 1.0
            ).finished().asDiagonal();
        }
    }
}

#endif
