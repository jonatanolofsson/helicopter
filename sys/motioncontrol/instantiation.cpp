#include <sys/motioncontrol/implementation.hpp>
#include <os/com/getSignal.hpp>

INSTANTIATE_SIGNAL(sys::motioncontrol::ModelDescription::ControlMessage);

namespace sys {
    namespace motioncontrol {
        template class MotionControl<motioncontrol::ModelDescription, motioncontrol::Controller, motioncontrol::ControlState, motioncontrol::ControlModel, motioncontrol::SystemState, motioncontrol::SystemModelDescription>;
    }
}
