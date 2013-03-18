#pragma once
#ifndef SYS_MOTIONCONTROL_HPP_
#define SYS_MOTIONCONTROL_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/observer/API.hpp>

namespace sys {
    namespace motioncontrol {
        template<typename ControllerType, typename ControlState, typename Model, typename Trigger>
        class MotionControl {
            public:
                typedef MotionControl<ControllerType, ControlState, Model, Trigger> Self;

            private:
                ControllerType controller;
                os::Dispatcher<Self, Trigger> dispatcher;

                void updateControl(const Trigger systemState);

            public:
                MotionControl();
        };
    }
}

#endif
