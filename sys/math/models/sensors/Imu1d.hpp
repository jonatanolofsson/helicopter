#pragma once
#ifndef SYS_MATH_MODELS_SENSORS_IMU1D_HPP_
#define SYS_MATH_MODELS_SENSORS_IMU1D_HPP_

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
            template<typename ModelDescription>
            struct Imu1d {
                typedef Imu1d Self;
                typedef typename ModelDescription::Scalar Scalar;
                enum state {
                    a = 0,
                    w  = 1,

                    nofMeasurements = 2
                };

                enum states {
                    acceleration = a,
                    rotational_velocity = w
                };

                typedef Matrix<Scalar, nofMeasurements, 1> MeasurementVector;
                typedef Matrix<Scalar, nofMeasurements, nofMeasurements> CovarianceMatrix;
                static MeasurementVector measurement(const typename ModelDescription::States& state) {
                    typedef typename ModelDescription::StateDescription states;
                    MeasurementVector m;
                    m[a] = state[states::a];
                    m[w] = state[states::w];

                    return m;
                }

                static CovarianceMatrix cov;
                static const CovarianceMatrix& covariance() { return cov; }

                static Matrix<Scalar, nofMeasurements, ModelDescription::nofStates>
                jacobian(const typename ModelDescription::States&) {
                    typedef typename ModelDescription::StateDescription states;
                    typedef Matrix<Scalar, nofMeasurements, ModelDescription::nofStates> JacobianMatrix;
                    JacobianMatrix J;
                    J.setZero();

                    J(a, states::a) = 1;
                    J(w, states::w)  = 1;

                    return J;
                }

                static MeasurementVector noise() {
                    return math::normalSample(covariance());
                }
            };

            template<typename ModelDescription> typename Imu1d<ModelDescription>::CovarianceMatrix
            Imu1d<ModelDescription>::cov = (Imu1d<ModelDescription>::MeasurementVector() <<
                1.0, 1.0
            ).finished().asDiagonal();
        }
    }
}

#endif
