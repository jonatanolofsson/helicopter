#include <sys/motioncontrol/implementation.hpp>
#include <os/com/getSignal.hpp>
#include <sys/com/MotionControlSignal.hpp>

INSTANTIATE_SIGNAL(sys::MotionControlSignal);

namespace sys {
    namespace motioncontrol {
        template class MotionControl<motioncontrol::Controller, motioncontrol::ControlState, motioncontrol::MotionModel, motioncontrol::SystemState>;
    }
}
