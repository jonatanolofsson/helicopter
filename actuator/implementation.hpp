#pragma once
#ifndef SYS_ACTUATOR_IMPLEMENTATION_HPP_
#define SYS_ACTUATOR_IMPLEMENTATION_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/settings.hpp>
#include <sys/com/MotionControlSignal.hpp>
#include <sys/com/CameraControlSignal.hpp>
#include <sys/com/Stm.hpp>
#include <sys/Actuator.hpp>

namespace sys {
    namespace actuator {
        using namespace stm;
        template<typename Serial>
        Actuator<Serial>::Actuator(Serial& stm_)
        : stm(stm_)
        , controlActuator(&Actuator<Serial>::actuateControl, this)
        {}


        template<typename Serial>
        void Actuator<Serial>::actuateControl(const MotionControlSignal u) {
            typedef ModelDescription::ControlDescription controls;
            Messages::ById<stm::Messages::controlMessage>::Type msg = {
                (S16)(u[controls::v] - 0.5 * wheelbase * u[controls::w]),
                (S16)(u[controls::v] + 0.5 * wheelbase * u[controls::w])
            };
            stm.template send<>(msg);
        }

        //~ template<typename Serial>
        //~ void Actuator<Serial>::actuateCamera(const CameraControlSignal c) {
            //~ Messages::ById<Messages::cameraControlMessage>::Type msg = {
                //~ (U16)(c[camera::horizontal]),
                //~ (U16)(c[camera::vertical])
            //~ };
            //~ maple.template send<>(msg);
        //~ }
    }
}

#endif
