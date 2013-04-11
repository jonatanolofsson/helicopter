#pragma once
#ifndef SYS_MATH_MODELS_SENSORS_IMU2D_HPP_
#define SYS_MATH_MODELS_SENSORS_IMU2D_HPP_

#include <sys/math/constants.hpp>
#include <sys/math/filtering/GaussianFilter.hpp>
#include <Eigen/Core>
#include <os/com/Dispatcher.hpp>
#include <sys/types.hpp>

namespace sys {
    namespace math {
        namespace models {
            using namespace Eigen;
            template<typename ModelDescription>
            struct Imu2d {
                typedef Imu2d Self;
                typedef typename ModelDescription::Scalar Scalar;
                enum state {
                    ax = 0,
                    ay = 1,

                    w  = 2,

                    nofMeasurements = 3
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

                    m[w] = state[states::w];

                    return m;
                }

                static CovarianceMatrix cov;
                static const CovarianceMatrix& covariance(const typename ModelDescription::States& state) { return cov; }

                static Matrix<Scalar, nofMeasurements, ModelDescription::nofStates>
                jacobian(const typename ModelDescription::States&) {
                    typedef typename ModelDescription::StateDescription states;
                    typedef Matrix<Scalar, nofMeasurements, ModelDescription::nofStates> JacobianMatrix;
                    JacobianMatrix J;
                    J.setZero();

                    J(ax, states::ax) = 1;
                    J(ay, states::ay) = 1;

                    J(wz, states::w)  = 1;

                    return J;
                }
            };

            template<typename ModelDescription> typename Imu2d<ModelDescription>::CovarianceMatrix
            Imu<ModelDescription>::cov = (Imu2d<ModelDescription>::MeasurementVector() <<
                1.0, 1.0, 1.0
            ).finished().asDiagonal();
        }
    }
}

#endif
