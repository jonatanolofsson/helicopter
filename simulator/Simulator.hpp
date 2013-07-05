#pragma once
#ifndef SYS_SIMULATOR_HPP_
#define SYS_SIMULATOR_HPP_

#include <os/type_traits.hpp>
#include <os/com/Dispatcher.hpp>
#include <sys/Observer.hpp>
#include <type_traits>

namespace sys {
    namespace simulator {
        template<typename MotionModel, typename... Sensors>
        class Simulator {
            public:
                typedef Simulator<MotionModel, Sensors...> Self;

                typename MotionModel::States state;
                os::Dispatcher<Self, typename MotionModel::ModelDescription::ControlMessage> d;

                template<typename Sensor>
                void yieldSensorReading() {
                    static math::GaussianMeasurement<Sensor> m;
                    m.z = Sensor::measurement(state);
                    os::yield(m);
                }

                template<typename... WSensors>
                void yieldSensorReadings() {
                    os::evalVariadic((yieldSensorReading<WSensors>(), 1)...);
                }

                Simulator();

                void simulate(const typename MotionModel::ModelDescription::ControlMessage);
        };
    }
}

#endif
