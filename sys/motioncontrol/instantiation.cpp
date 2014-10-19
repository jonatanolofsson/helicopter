#include <sys/motioncontrol/implementation.hpp>
#include <os/com/getSignal.hpp>

INSTANTIATE_SIGNAL(sys::motioncontrol::ControlMessage);

namespace sys {
    namespace motioncontrol {
        template class MOTIONCONTROL_CLASS;
    }
}
