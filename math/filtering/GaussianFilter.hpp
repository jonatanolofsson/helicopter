#pragma once
#ifndef SYS_MATH_FILTERING_GAUSSIAN_FILTER_HPP_
#define SYS_MATH_FILTERING_GAUSSIAN_FILTER_HPP_

#include <os/mem/ProtectedData.hpp>
#include <sys/math/statistics.hpp>
#include <sys/math/filtering.hpp>
#include <sys/math/control.hpp>
#include <sys/types.hpp>

namespace sys {
    namespace math {
        template<typename ModelDescription>
        struct GaussianFilter : public os::ProtectedClass {
            typedef typename ModelDescription::Scalar Scalar;
            typedef typename ModelDescription::States Reference;
            typedef GaussianFilter<ModelDescription> Self;
            typedef typename Covariance<typename ModelDescription::Scalar, ModelDescription::nofStates>::type CovarianceMatrix;
            typedef typename ModelDescription::States States;
            CovarianceMatrix covariance;
            States state;

            explicit GaussianFilter() {
                state.setZero();
                covariance.setZero();
            }

            States noise() {
                return math::normalSample(covariance);
            }
        };

        template<typename Sensor_>
        struct GaussianMeasurement {
            typedef Sensor_ Sensor;
            typename Sensor::MeasurementVector z;
            Matrix<typename Sensor::Scalar, Sensor::nofMeasurements, Sensor::nofMeasurements> R;
        };
    }
}

#endif
