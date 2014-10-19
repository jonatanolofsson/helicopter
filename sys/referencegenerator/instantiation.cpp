#include <sys/ReferenceGenerator.hpp>
#include <sys/referencegenerator/implementation.hpp>
#include <os/com/getSignal.hpp>

INSTANTIATE_SIGNAL(sys::referencegenerator::ReferenceMessage);

namespace sys {
    namespace referencegenerator {
        template class ReferenceGenerator<ReferenceMessage, Trigger>;
    }
}
