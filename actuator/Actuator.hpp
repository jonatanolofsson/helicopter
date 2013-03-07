#ifndef SYS_ACTUATOR_HPP_
#define SYS_ACTUATOR_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/settings.hpp>
#include <sys/com/MotionControlSignal.hpp>
#include <sys/com/CameraControlSignal.hpp>
#include <sys/com/MapleMessages.hpp>

namespace sys {
    namespace actuator {
        template<typename Serial>
        class Actuator {
            private:
                void actuateControl(const MotionControlSignal u) {
                    MapleMessages::Message<MapleMessages::controlMessage>::Type msg = {
                        {
                            (U16)(u[control::servo[0]]),
                            (U16)(u[control::servo[1]]),
                            (U16)(u[control::servo[2]])
                        },
                        (U16)(u[control::rpm])
                    };
                    maple.template send<>(msg);
                }

                void actuateCamera(const CameraControlSignal c) {
                    MapleMessages::Message<MapleMessages::cameraControlMessage>::Type msg = {
                        (U16)(c[camera::horizontal]),
                        (U16)(c[camera::vertical])
                    };
                    maple.template send<>(msg);
                }

                Serial& maple;

                os::Dispatcher<Actuator, MotionControlSignal> controlActuator;
                os::Dispatcher<Actuator, CameraControlSignal> cameraActuator;

            public:
                explicit Actuator(Serial& maple_)
                : maple(maple_)
                , controlActuator(&Actuator<Serial>::actuateControl, this)
                , cameraActuator(&Actuator<Serial>::actuateCamera, this)
                {}
        };
    }
}

#endif
