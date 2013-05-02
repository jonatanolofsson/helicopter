#pragma once
#ifndef SYS_MATH_MODELS_SENSORS_ULTRASOUNDINMAP_HPP_
#define SYS_MATH_MODELS_SENSORS_ULTRASOUNDINMAP_HPP_

#include <Eigen/Core>
#include <sys/math/base.hpp>
#include <cmath>
#include <sys/math/statistics.hpp>

namespace sys {
    namespace math {
        namespace models {
            using namespace Eigen;

            template<typename S>
            struct UltrasoundSensor {
                typedef Matrix<S, 2, 1> Position;
                Position position;
                S angle;
            };

            template<typename ModelDescription, unsigned N, typename Map>
            struct UltrasoundInMap {
                typedef typename ModelDescription::Scalar Scalar;
                typedef typename ModelDescription::States States;
                typedef UltrasoundSensor<Scalar> SensorType;
                typedef typename ModelDescription::StateDescription states;
                static const unsigned nofMeasurements = N;
                typedef Matrix<Scalar, nofMeasurements, 1> MeasurementVector;
                typedef Matrix<Scalar, nofMeasurements, nofMeasurements> CovarianceMatrix;

                SensorType sensors[N];
                const Map map;

                States getSensorPose(const States& state, const UltrasoundSensor<Scalar>& sensor) {
                    auto x = state;
                    x.template segment<2>(states::position) += Rotation2D<Scalar>(x[states::th]) * sensor.position;
                    x(states::th) += sensor.angle;
                    return x;
                }

                MeasurementVector measurement(const States& state) {
                    MeasurementVector m;
                    for(unsigned i = 0; i < N; ++i) {
                        m(i) = map.template getIntersection<ModelDescription>(getSensorPose(state, sensors[i]));
                    }
                    //~ std::cout << "Measurement state: " << state.transpose() << std::endl;
                    //~ std::cout << "Measurement: " << m.transpose() << std::endl;
                    return m;
                }

                static CovarianceMatrix cov;
                static const CovarianceMatrix& covariance() { return cov; }

                static MeasurementVector noise() {
                    return math::normalSample(covariance());
                }
            };

            template<typename ModelDescription, unsigned N, typename Map> typename UltrasoundInMap<ModelDescription, N, Map>::CovarianceMatrix
            UltrasoundInMap<ModelDescription, N, Map>::cov = UltrasoundInMap<ModelDescription, N, Map>::MeasurementVector::Constant(0.05).asDiagonal();
        }
    }
}

#endif
