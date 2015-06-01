#pragma once

#include <os/com/Dispatcher.hpp>

namespace sys {
    namespace speedcontrol {
        template<typename GlobalFilter, typename Algorithm, typename MotionModel, typename SystemStateMessage, typename Reference, typename ControlMessage>
        class SpeedControl {
            public:
                typedef SpeedControl<GlobalFilter, Algorithm, MotionModel, SystemStateMessage, Reference, ControlMessage> Self;

            private:
                GlobalFilter& filter;
                typedef typename SystemStateMessage::StateVector SystemStateVector;
                typedef typename SystemStateMessage::States SystemStates;

                Algorithm controller;
                os::Dispatcher<Self, SystemStateMessage, Reference> dispatcher;
                typename Algorithm::Controls control;

                void updateControl(const SystemStateMessage, const Reference);

            public:
                MotionControl(GlobalFilter&);
        };
    }
}
