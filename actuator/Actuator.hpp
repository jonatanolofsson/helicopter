#ifndef SYS_ACTUATOR_HPP_
#define SYS_ACTUATOR_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/com/MotionControlSignal.hpp>
#include <sys/com/CameraControlSignal.hpp>

namespace sys {
    namespace actuator {
        template<typename Serial>
        class Actuator {
            private:
                void actuateControl(const MotionControlSignal u);
                void actuateCamera(const CameraControlSignal c);

                Serial& maple;

                os::Dispatcher<Actuator, MotionControlSignal> controlActuator;
                os::Dispatcher<Actuator, CameraControlSignal> cameraActuator;

            public:
                explicit Actuator(Serial& maple_);
        };
    }
}

#endif
