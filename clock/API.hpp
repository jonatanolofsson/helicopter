#ifndef SYS_CLOCK_API_HPP_
#define SYS_CLOCK_API_HPP_

#include <sys/types.hpp>
#include <sys/clock/Clock.hpp>
#include <chrono>

namespace sys {
    namespace clock {
        typedef std::chrono::high_resolution_clock SystemClock;
        static const std::chrono::microseconds realTimePerTick(0);
    }

    typedef clock::Clock Clock;
}

#endif
