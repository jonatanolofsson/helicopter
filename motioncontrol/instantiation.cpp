#include <sys/motioncontrol/implementation.hpp>
#include <os/com/getSignal.hpp>

INSTANTIATE_SIGNAL(sys::motioncontrol::Controls);

namespace sys {
    namespace motioncontrol {
        template class MotionControl<motioncontrol::Controller, motioncontrol::ControlState, motioncontrol::ControlModel, motioncontrol::SystemState>;
    }
}
