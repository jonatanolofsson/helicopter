#include <os/com/getSignal.hpp>

#include <sys/towercontrol/API.hpp>
#include <sys/towercontrol/implementation.hpp>

namespace sys {
    namespace towercontrol {
        template class TowerControl<SerialLink>;
    }
}
