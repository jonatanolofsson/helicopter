#pragma once
#ifndef SYS_MATH_MODELS_SENSORS_IMU_HPP_
#define SYS_MATH_MODELS_SENSORS_IMU_HPP_

#include <sys/math/constants.hpp>
#include <sys/math/filtering/GaussianFilter.hpp>
#include <Eigen/Core>
#include <os/com/Dispatcher.hpp>
#include <sys/types.hpp>

namespace sys {
    namespace models {
        namespace sensors {
            using namespace Eigen;
            struct Imu {
                typedef Imu Self;
                typedef sys::Scalar Scalar;
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
                template<typename T>
                static MeasurementVector measurement(const T& filter) {
                    typedef typename T::Model::StateDescription states;
                    MeasurementVector m;
                    m[ax] = filter.state[states::ax];
                    m[ay] = filter.state[states::ay];
                    m[az] = filter.state[states::az];

                    m[wx] = filter.state[states::wx];
                    m[wy] = filter.state[states::wy];
                    m[wz] = filter.state[states::wz];

                    return m;
                }

                template<typename T>
                static Matrix<typename T::Scalar, nofMeasurements, T::Model::nofStates>
                jacobian(const T&) {
                    typedef typename T::Model::StateDescription states;
                    typedef Matrix<Scalar, nofMeasurements, states::nofStates> JacobianMatrix;
                    JacobianMatrix J;
                    J.setZero();

                    J(ax,states::ax) = 1;
                    J(ay,states::ay) = 1;
                    J(az,states::az) = 1;

                    J(wx,states::wx) = 1;
                    J(wy,states::wy) = 1;
                    J(wz,states::wz) = 1;

                    return J;
                }
            };
        }
    }
}

#endif
