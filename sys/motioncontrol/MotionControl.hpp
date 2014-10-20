#pragma once
#ifndef SYS_MOTIONCONTROL_HPP_
#define SYS_MOTIONCONTROL_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/Observer.hpp>

namespace sys {
    namespace motioncontrol {
        template<typename Algorithm, typename MotionModel, typename SystemStateMessage, typename Reference, typename ControlMessage>
        class MotionControl {
            public:
                typedef MotionControl<Algorithm, MotionModel, SystemStateMessage, Reference, ControlMessage> Self;

            private:
                typedef typename SystemStateMessage::StateVector SystemStateVector;
                typedef typename SystemStateMessage::States SystemStates;

                Algorithm controller;
                os::Dispatcher<Self, SystemStateMessage, Reference> dispatcher;
                typename Algorithm::Controls control;

                void updateControl(const SystemStateMessage, const Reference);

            public:
                MotionControl();
        };
    }
}

#endif
