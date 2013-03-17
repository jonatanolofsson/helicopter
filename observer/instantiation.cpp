#include <sys/observer/API.hpp>
#include <sys/observer/implementation.hpp>

namespace sys {
    namespace observer {
        template class Observer<FilterType, MotionModel, FilterState, TriggerType>;
    }
}
