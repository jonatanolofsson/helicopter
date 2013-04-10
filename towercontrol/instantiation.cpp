#include <os/com/getSignal.hpp>
#include <sys/com/MotionControlSignal.hpp>
#include <sys/com/CameraControlSignal.hpp>

#include <sys/actuator/API.hpp>
#include <sys/actuator/implementation.hpp>

INSTANTIATE_SIGNAL(sys::MotionControlSignal);
INSTANTIATE_SIGNAL(sys::CameraControlSignal);

namespace sys {
    namespace actuator {
        template class Actuator<SerialLink>;
    }
}
