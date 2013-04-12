#pragma once
#ifndef SYS_MATH_MODELS_SENSORS_IMU_HPP_
#define SYS_MATH_MODELS_SENSORS_IMU_HPP_

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
            struct Imu {
                typedef Imu Self;
                typedef typename ModelDescription::Scalar Scalar;
                enum state {
                    ax = 0,
                    ay = 1,
                    az = 2,

                    wx = 3,
                    wy = 4,
                    wz = 5,

                    nofMeasurements = 6
                };

                enum states {
                    acceleration = ax,
                    rotational_velocity = wx
                };

                typedef Matrix<Scalar, nofMeasurements, 1> MeasurementVector;
                typedef Matrix<Scalar, nofMeasurements, nofMeasurements> CovarianceMatrix;
                static MeasurementVector measurement(const typename ModelDescription::States& state) {
                    typedef typename ModelDescription::StateDescription states;
                    MeasurementVector m;
                    m[ax] = state[states::ax];
                    m[ay] = state[states::ay];
                    m[az] = state[states::az];

                    m[wx] = state[states::wx];
                    m[wy] = state[states::wy];
                    m[wz] = state[states::wz];

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

                    J(ax, states::ax) = 1;
                    J(ay, states::ay) = 1;
                    J(az, states::az) = 1;

                    J(wx, states::wx) = 1;
                    J(wy, states::wy) = 1;
                    J(wz, states::wz) = 1;

                    return J;
                }

                static MeasurementVector noise() {
                    return math::normalSample(covariance());
                }
            };

            template<typename ModelDescription> typename Imu<ModelDescription>::CovarianceMatrix
            Imu<ModelDescription>::cov = (Imu<ModelDescription>::MeasurementVector() <<
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0
            ).finished().asDiagonal();
        }
    }
}

#endif
