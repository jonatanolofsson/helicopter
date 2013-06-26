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
            typedef typename Covariance<typename ModelDescription::Scalar, ModelDescription::nofStates>::Type CovarianceMatrix;
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

        template<typename Sensor_, bool CALL_WITH_OBJECT = false>
        struct GaussianMeasurement {
            typedef Sensor_ Sensor;
            typedef typename Sensor::MeasurementVector MeasurementVector;
            MeasurementVector z;
            Matrix<typename Sensor::Scalar, Sensor::nofMeasurements, Sensor::nofMeasurements> R;
            template<typename States>
            MeasurementVector measurement(const States& state) const {
                return Sensor::measurement(state);
            }
            GaussianMeasurement() {
                z.setZero();
                R = Sensor::covariance();
            }
        };

        template<typename Sensor_>
        struct GaussianMeasurement<Sensor_, true> {
            typedef Sensor_ Sensor;
            typedef typename Sensor::MeasurementVector MeasurementVector;
            MeasurementVector z;
            Sensor* sensor;
            Matrix<typename Sensor::Scalar, Sensor::nofMeasurements, Sensor::nofMeasurements> R;
            GaussianMeasurement() : sensor(nullptr) 
            {
                z.setZero();
                R = Sensor::covariance();
            }
            template<typename States>
            MeasurementVector measurement(const States& state) const {
                assert(sensor);
                return sensor->measurement(state);
            }
        };
    }
}

#endif
