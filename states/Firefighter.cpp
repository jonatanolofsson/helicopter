#include <sys/states/Firefighter.hpp>

namespace sys {
    namespace states {
        void Firefighter::kill() {
            std::unique_lock<std::mutex> l(m);
            dying = true;
            waitingForDeath.notify_all();
        }

        void Firefighter::wait() {
            std::unique_lock<std::mutex> l(m);
            while(!dying) waitingForDeath.wait(l);
        }

        void Firefighter::react(const events::Dying&) {
            kill();
        }
    }
}
