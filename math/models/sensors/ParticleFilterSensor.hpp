#pragma once
#ifndef SYS_MATH_MODELS_SENSORS_PARTICLEFILTERSENSOR_HPP_
#define SYS_MATH_MODELS_SENSORS_PARTICLEFILTERSENSOR_HPP_

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
            struct ParticleFilterSensor {
                typedef ParticleFilterSensor<ModelDescription> Self;
                typedef typename ModelDescription::Scalar Scalar;
                enum state {
                    x = 0,
                    y = 1,
                    th = 2,

                    nofMeasurements = 3
                };

                enum states {
                    position = x,
                    orientation = th
                };

                typedef Matrix<Scalar, nofMeasurements, 1> MeasurementVector;
                typedef Matrix<Scalar, nofMeasurements, nofMeasurements> CovarianceMatrix;
                static MeasurementVector measurement(const typename ModelDescription::States& state) {
                    typedef typename ModelDescription::StateDescription states;
                    MeasurementVector m;
                    m[x] = state[states::x];
                    m[y] = state[states::y];
                    m[th] = state[states::th];

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

                    J(x, states::x) = 1;
                    J(y, states::y) = 1;
                    J(th, states::th) = 1;

                    return J;
                }

                static MeasurementVector noise() {
                    return math::normalSample(covariance());
                }
            };

            template<typename ModelDescription> typename ParticleFilterSensor<ModelDescription>::CovarianceMatrix
            ParticleFilterSensor<ModelDescription>::cov = (ParticleFilterSensor<ModelDescription>::MeasurementVector() <<
                1.0, 1.0, 1.0
            ).finished().asDiagonal();
        }
    }
}

#endif
