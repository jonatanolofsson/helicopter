#include <sys/motioncontrol/implementation.hpp>
#include <os/com/getSignal.hpp>

INSTANTIATE_SIGNAL(sys::motioncontrol::SpeedControlMessage);

namespace sys {
    namespace speedcontrol {
        template class SPEEDCONTROL_CLASS;
    }
}
