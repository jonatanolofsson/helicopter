#include <sys/Clock.hpp>
#include <ctime>
#include <chrono>
#include <os/com/Dispatcher.hpp>
#include <iostream>

namespace sys {
    namespace clock {
        Clock::Clock()
        : d(&Clock::tick, this)
        , nextInvokation(SystemClock::now() + realTimePerTick)
        {
            os::yield(time);
        }

        void Clock::tick(const os::Jiffy) {
            std::this_thread::sleep_until(nextInvokation);
            os::yield(++time);
            nextInvokation = SystemClock::now() + realTimePerTick;
        }
    }
}
