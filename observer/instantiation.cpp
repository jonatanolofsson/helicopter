#include <sys/Observer.hpp>
#include <sys/observer/implementation.hpp>
#include <os/com/getSignal.hpp>

INSTANTIATE_SIGNAL(sys::observer::Filter::States);

namespace sys {
    namespace observer {
        template class Observer<Algorithm, Filter, MotionModel, TriggerType>;
    }
}
