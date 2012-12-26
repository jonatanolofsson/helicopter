#include <sys/settings.hpp>
#include <sys/Observer/Observer.hpp>
#include <os/clock.hpp>
#include <os/com/Dispatcher.hpp>

namespace sys {
    SystemState system_state(sys::settings::dT);

    namespace observer {
        void timeUpdate(const os::SystemTime) {
            filter::timeUpdate<MotionModel>(system_state);
        }

        namespace {
            os::Dispatcher<os::SystemTime> e(timeUpdate);

            __attribute__((constructor)) void init() {
                StateDescription::initialize(system_state);
            }
        }
    }
}
