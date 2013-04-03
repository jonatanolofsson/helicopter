#pragma once
#ifndef SYS_MATH_MODELS_SENSORS_ULTRASOUNDINMAP_HPP_
#define SYS_MATH_MODELS_SENSORS_ULTRASOUNDINMAP_HPP_

#include <Eigen/Core>
#include <sys/math/base.hpp>
#include <cmath>

namespace sys {
    namespace models {
        namespace sensors {
            using namespace Eigen;
            template<typename Map, unsigned N>
            class UltrasoundInMap {
                public:
                    typedef Matrix<Scalar, N, 1> MeasurementVector;
                    static MeasurementVector variance;
                    Scalar sensorAngles[N];
                    Map map;

                    UltrasoundInMap(Scalar&& angles) : sensorAngles(angles) {
                        variance << 0.5, 0.5, 0.5;
                    }

                    MeasurementVector measurement(const ParticleState& state) {
                        MeasurementVector m;
                        for(unsigned i = 0; i < N; ++i) {
                            m(i) = map.getIntersection(state, sensorAngles[i]);
                        }
                        return m;
                    }
            };
        }
    }
}

#endif
