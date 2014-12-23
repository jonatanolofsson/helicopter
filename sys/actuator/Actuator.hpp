#pragma once

#include <os/com/Dispatcher.hpp>

namespace sys {
    namespace actuator {
        template<typename Serial, typename ControlSignal>
        class Actuator {
            public:
                typedef Actuator<Serial, ControlSignal> Self;

            private:
                void actuateControl(const ControlSignal) {}

                Serial& mcu;

                os::Dispatcher<Self, ControlSignal> controlActuator;

                Actuator();

            public:
                explicit Actuator(Serial& mcu_)
                : mcu(mcu_)
                , controlActuator(&Self::actuateControl, this)
                {}
        };
    }
}
