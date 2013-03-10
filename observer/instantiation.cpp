#include <sys/observer/API.hpp>
#include <sys/observer/implementation.hpp>
#include <os/com/getSignal.hpp>

INSTANTIATE_SIGNAL(sys::observer::sensors::GPS);

namespace sys {
    namespace observer {
        template class Observer<FilterType, MotionModel, FilterState, TriggerType>;
    }
}
