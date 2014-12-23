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
        template<typename Serial, typename ControlSignal>
        void Actuator<Serial>::actuateControl(const ControlSignal u) {
            //mcu.template send<>(msg);
        }
    }
}

#endif
