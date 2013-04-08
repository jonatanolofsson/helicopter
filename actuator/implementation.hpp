#pragma once
#ifndef SYS_ACTUATOR_IMPLEMENTATION_HPP_
#define SYS_ACTUATOR_IMPLEMENTATION_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/settings.hpp>
#include <sys/com/MotionControlSignal.hpp>
#include <sys/com/CameraControlSignal.hpp>
#include <sys/com/Maple.hpp>
#include <sys/actuator/API.hpp>

namespace sys {
    namespace actuator {
        using namespace maple;
        template<typename Serial>
        Actuator<Serial>::Actuator(Serial& maple_)
        : maple(maple_)
        , controlActuator(&Actuator<Serial>::actuateControl, this)
        , cameraActuator(&Actuator<Serial>::actuateCamera, this)
        {}


        template<typename Serial>
        void Actuator<Serial>::actuateControl(const MotionControlSignal u) {
            Messages::ById<Messages::controlMessage>::Type msg = {
                {
                    (U16)(u[control::servo[0]]),
                    (U16)(u[control::servo[1]]),
                    (U16)(u[control::servo[2]])
                },
                (U16)(u[control::rpm])
            };
            maple.template send<>(msg);
        }

        template<typename Serial>
        void Actuator<Serial>::actuateCamera(const CameraControlSignal c) {
            Messages::ById<Messages::cameraControlMessage>::Type msg = {
                (U16)(c[camera::horizontal]),
                (U16)(c[camera::vertical])
            };
            maple.template send<>(msg);
        }
    }
}

#endif
