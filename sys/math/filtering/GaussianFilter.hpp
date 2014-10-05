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
        template<typename StateDescription>
        struct GaussianFilter : public os::ProtectedClass {
            typedef typename StateDescription::Scalar Scalar;
            typedef GaussianFilter<StateDescription> Self;
            typedef math::internal::Covariance<typename StateDescription::Scalar, StateDescription::nofStates> CovarianceMatrix;
            typedef typename StateDescription::StateVector StateVector;
            CovarianceMatrix covariance;
            StateVector state;

            explicit GaussianFilter() {
                state.setZero();
                covariance.setZero();
            }

            StateVector noise() {
                return math::normalSample(covariance);
            }
        };

        template<typename Sensor_, bool CALL_WITH_OBJECT = false>
        struct GaussianMeasurement {
            typedef Sensor_ Sensor;
            typedef typename Sensor::MeasurementVector MeasurementVector;
            MeasurementVector z;
            Matrix<typename Sensor::Scalar, Sensor::nofMeasurements, Sensor::nofMeasurements> R;
            template<typename StateVector>
            MeasurementVector measurement(const StateVector& state) const {
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
            template<typename StateVector>
            MeasurementVector measurement(const StateVector& state) const {
                assert(sensor);
                return sensor->measurement(state);
            }
        };
    }
}

#endif
