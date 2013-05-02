#include <sys/states/Helicopter.hpp>

namespace sys {
    namespace states {
        void Helicopter::kill() {
            std::unique_lock<std::mutex> l(m);
            dying = true;
            waitingForDeath.notify_all();
        }

        void Helicopter::wait() {
            std::unique_lock<std::mutex> l(m);
            while(!dying) waitingForDeath.wait(l);
        }

        void Helicopter::react(const events::Dying&) {
            kill();
        }
    }
}
