#pragma once
#ifndef SYS_MOTIONCONTROL_HPP_
#define SYS_MOTIONCONTROL_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/Observer.hpp>

namespace sys {
    namespace motioncontrol {
        template<typename ModelDescription, typename ControllerType, typename ControlState, typename ControlModel, typename SystemState>
        class MotionControl {
            public:
                typedef MotionControl<ModelDescription, ControllerType, ControlState, ControlModel, SystemState> Self;
                typedef typename ModelDescription::ReferenceMessage ReferenceMessage;

            private:
                ControllerType controller;
                os::Dispatcher<Self, SystemState, ReferenceMessage> dispatcher;

                void updateControl(const SystemState systemState, const ReferenceMessage referenceMessage);

            public:
                MotionControl();
        };
    }
}

#endif
