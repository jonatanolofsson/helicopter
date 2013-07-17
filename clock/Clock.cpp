#include <sys/Clock.hpp>
#include <ctime>
#include <chrono>
#include <os/com/Dispatcher.hpp>
#include <iostream>

#include <sys/States.hpp>

namespace sys {
    namespace clock {
        Clock::Clock()
        : d(&Clock::tick, this)
        , nextInvokation(SystemClock::now() + realTimePerTick)
        {}

        void Clock::start() {
            os::startTime();
        }

        void Clock::stop() {
            os::stopTime();
        }

        Clock::~Clock() {
            stop();
        }

        void Clock::tick(const os::Jiffy) {
            std::this_thread::sleep_until(nextInvokation);
            if(time.value >= 2000) {
                stop();
                killStateMachine();
                return;
            }
            os::yield(++time);
            nextInvokation = SystemClock::now() + realTimePerTick;
        }
    }
}
