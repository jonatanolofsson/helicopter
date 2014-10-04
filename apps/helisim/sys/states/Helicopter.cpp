#include <sys/states/Helicopter.hpp>
#include <iostream>

namespace sys {
    namespace states {
        void Helicopter::kill() {
            std::unique_lock<std::mutex> l(m);
            dying = true;
            waitingForDeath.notify_all();
        }

        void Helicopter::wait() {
            std::unique_lock<std::mutex> l(m);
            LOG_EVENT(typeid(Self).name(), 0, "Waiting to die");
            while(!dying) {
                waitingForDeath.wait(l);
                LOG_EVENT(typeid(Self).name(), 0, "Woke up to die");
            }
        }

        void Helicopter::react(const events::Dying&) {
            kill();
        }
    }
}
