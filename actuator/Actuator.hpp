#pragma once
#ifndef SYS_ACTUATOR_HPP_
#define SYS_ACTUATOR_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/motioncontrol/API.hpp>
#include <sys/actuator/API.hpp>

namespace sys {
    namespace actuator {
        template<typename Serial>
        class Actuator {
            public:
                typedef Actuator<Serial> Self;

            private:
                void actuateControl(const MotionControlSignal u);

                Serial& stm;

                os::Dispatcher<Self, MotionControlSignal> controlActuator;

                Actuator();

            public:
                explicit Actuator(Serial&);
        };
    }
}
#endif
