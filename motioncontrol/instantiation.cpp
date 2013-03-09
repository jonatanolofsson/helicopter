#include <sys/motioncontrol/implementation.hpp>

namespace sys {
    namespace motioncontrol {
        template class MotionControl<Controller, ControlState, MotionModel, SystemState>;
    }
}
