#pragma once

#include <os/type_traits.hpp>
#include <os/com/Dispatcher.hpp>
#include <sys/Observer.hpp>
#include <type_traits>
#include <os/utils/eventlog.hpp>

namespace sys {
    namespace simulator {
        template<typename GlobalFilter, typename MotionModel, typename ControlMessage, typename... Sensors>
        struct Simulator {
            GlobalFilter& filter;

            typedef Simulator<GlobalFilter, MotionModel, ControlMessage, Sensors...> Self;
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
                //LOG_EVENT(typeid(Self).name(), 50, "Sensor " << os::demangle(typeid(SensorMeasurement).name()) << ": " << m.z.transpose());
                os::yield(m);
            }

            template<typename... WSensors>
            void yieldSensorReadings() {
                //LOG_EVENT(typeid(Self).name(), 50, "Yielding values from " << sizeof...(WSensors) << " sensors");
                os::evalVariadic((yieldSensorReading<WSensors>(), 1)...);
            }

            explicit Simulator(GlobalFilter&);

            void simulate(const ControlMessage);
        };
    }
}

