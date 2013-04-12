#pragma once
#ifndef SYS_MATH_MODELS_SENSORS_MOUSE_HPP_
#define SYS_MATH_MODELS_SENSORS_MOUSE_HPP_

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
            struct Mouse {
                typedef Mouse<ModelDescription> Self;
                typedef typename ModelDescription::Scalar Scalar;
                enum state {
                    v = 0,
                    nofMeasurements = 1
                };

                enum states {
                    velocity = v
                };

                typedef Matrix<Scalar, nofMeasurements, 1> MeasurementVector;
                typedef Matrix<Scalar, nofMeasurements, nofMeasurements> CovarianceMatrix;
                static MeasurementVector measurement(const typename ModelDescription::States& state) {
                    typedef typename ModelDescription::StateDescription states;
                    MeasurementVector m;
                    m[v] = state[states::v];
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
                    J(v, states::v) = 1;

                    return J;
                }

                static MeasurementVector noise() {
                    return math::normalSample(covariance());
                }
            };

            template<typename ModelDescription> typename Mouse<ModelDescription>::CovarianceMatrix
            Mouse<ModelDescription>::cov = (Mouse<ModelDescription>::MeasurementVector() <<
                1.0
            ).finished().asDiagonal();
        }
    }
}

#endif
