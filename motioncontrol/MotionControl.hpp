#pragma once
#ifndef SYS_MOTIONCONTROL_HPP_
#define SYS_MOTIONCONTROL_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/observer/API.hpp>

namespace sys {
    namespace motioncontrol {
        template<typename ControllerType, typename ControlState, typename ControlModel, typename SystemState>
        class MotionControl {
            public:
                typedef MotionControl<ControllerType, ControlState, ControlModel, SystemState> Self;

            private:
                ControllerType controller;
                os::Dispatcher<Self, SystemState> dispatcher;

                void updateControl(const SystemState systemState);

            public:
                MotionControl();
        };
    }
}

#endif
