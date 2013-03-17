#include <sys/clock/API.hpp>
#include <ctime>
#include <chrono>
#include <os/com/Dispatcher.hpp>

namespace sys {
    namespace clock {
        Clock::Clock() {
            t = std::thread(&Clock::run, this);
        }

        void Clock::start() {
            dying = false;
            time.value = 0;
            os::yield(time);
        }

        void Clock::stop() {
            dying = true;
        }

        void Clock::run() {
            while(!dying) {
                tick(os::Via<os::Jiffy>::value());
                std::this_thread::sleep_for(realTimePerTick);
            }
        }

        void Clock::tick(const os::Jiffy) {
            os::yield(++time);
        }

        Clock::~Clock() {
            stop();
            t.join();
        }
    }
}
