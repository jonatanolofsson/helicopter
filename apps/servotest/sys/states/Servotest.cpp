#include <sys/states/Servotest.hpp>

namespace sys {
    namespace states {
        void Servotest::kill() {
            std::unique_lock<std::mutex> l(m);
            dying = true;
            waitingForDeath.notify_all();
        }

        void Servotest::wait() {
            std::unique_lock<std::mutex> l(m);
            while(!dying) waitingForDeath.wait(l);
        }

        void Servotest::react(const events::Dying&) {
            kill();
        }
    }
}
