#include <sys/Observer.hpp>
#include <sys/observer/implementation.hpp>
#include <os/com/getSignal.hpp>

INSTANTIATE_SIGNAL(sys::observer::Filter::States);
INSTANTIATE_SIGNAL(sys::observer::sensors::Gps);

namespace sys {
    namespace observer {
        template class OBSERVER_CLASS;
    }
}
