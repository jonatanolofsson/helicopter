#pragma once
#ifndef SYS_SIMULATOR_HPP_
#define SYS_SIMULATOR_HPP_

#include <os/type_traits.hpp>
#include <os/com/Dispatcher.hpp>
#include <sys/Observer.hpp>
#include <type_traits>
#include <os/utils/eventlog.hpp>

namespace sys {
    namespace simulator {
        template<typename MotionModel, typename ControlMessage, typename... Sensors>
        class Simulator {
            public:
                typedef Simulator<MotionModel, ControlMessage, Sensors...> Self;
                typedef typename MotionModel::States States;

                typename MotionModel::StateVector state;
                os::Dispatcher<Self, ControlMessage> d;

                template<typename SensorMeasurement>
                void yieldSensorReading() {
                    static SensorMeasurement m;
                    static unsigned callCount = 0;
                    if ((++callCount % (settings::systemFrequency / SensorMeasurement::Sensor::frequency)) != 0) {
                        return;
                    }
                    m.z = SensorMeasurement::Sensor::template measurement<States>(state);
                    LOG_EVENT(typeid(Self).name(), 50, "Sensor " << os::demangle(typeid(SensorMeasurement).name()) << ": " << m.z.transpose());
                    os::yield(m);
                }

                template<typename... WSensors>
                void yieldSensorReadings() {
                    LOG_EVENT(typeid(Self).name(), 50, "Yielding values from " << sizeof...(WSensors) << " sensors");
                    os::evalVariadic((yieldSensorReading<WSensors>(), 1)...);
                }

                Simulator();

                void simulate(const ControlMessage);
        };
    }
}

#endif
