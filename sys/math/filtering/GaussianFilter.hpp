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
        template<typename States_>
        struct GaussianFilter : public os::ProtectedClass {
            typedef States_ States;
            typedef GaussianFilter<States> Self;
            typedef math::internal::Covariance<States::nofStates> CovarianceMatrix;
            typedef typename States::StateVector StateVector;
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
            typedef typename Sensor::States States;
            typedef typename Sensor::States::StateVector StateVector;
            StateVector z;
            Matrix<Scalar, Sensor::nofStates, Sensor::nofStates> R;
            template<typename ExternalStates>
            StateVector measurement(const typename ExternalStates::StateVector& state) const {
                return Sensor::template measurement<ExternalStates>(state);
            }
            GaussianMeasurement() {
                z.setZero();
                R = Sensor::covariance();
            }
        };

        template<typename Sensor_>
        struct GaussianMeasurement<Sensor_, true> {
            typedef Sensor_ Sensor;
            typedef typename Sensor::States States;
            typedef typename Sensor::States::StateVector StateVector;
            StateVector z;
            Sensor* sensor;
            Matrix<Scalar, Sensor::nofStates, Sensor::nofStates> R;
            GaussianMeasurement() : sensor(nullptr)
            {
                z.setZero();
                R = Sensor::covariance();
            }
            template<typename ExternalStates>
            StateVector measurement(const typename ExternalStates::StateVector& state) const {
                assert(sensor);
                return sensor->measurement(state);
            }
        };
    }
}

#endif
