#pragma once
#ifndef SYS_SENSORHUB_SENSORS_STMIR_HPP_
#define SYS_SENSORHUB_SENSORS_STMIR_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/com/Stm.hpp>
#include <sys/math/filtering.hpp>
#include <sys/math/models.hpp>
#include <sys/math/states.hpp>
#include <sys/math/models/Map.hpp>

#include <string>

namespace sys {
    namespace sensorhub {
        namespace stmir {
            typedef math::models::S2DPose StateDescription;
            typedef math::models::CVW ControlDescription;
            typedef math::models::Description<StateDescription, ControlDescription> ModelDescription;

            typedef sys::math::models::UltrasoundInMap<ModelDescription, 4, sys::math::Map<14>> Sensor;
            typedef sys::math::GaussianMeasurement<Sensor, true> DistanceMeasurement;
        }

        class StmIR {
            private:
                os::Dispatcher<StmIR, stm::SensorMessage> d;
                stmir::DistanceMeasurement m;

            public:
                StmIR();
                void handleMessage(const stm::SensorMessage);

                // Remove after testing
                bool up;
        };
    }
}

#endif
