#include <sys/Servomover.hpp>
#include <sys/servomover/implementation.hpp>

namespace sys {
    namespace servomover {
        template class Servomover<TriggerType, SerialLink>;
    }
}
