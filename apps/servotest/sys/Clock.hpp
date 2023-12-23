#pragma once

#include <sys/types.hpp>
#include <chrono>
#include <sys/settings.hpp>

namespace sys {
    namespace clock {
        typedef std::chrono::high_resolution_clock SystemClock;
        static const std::chrono::microseconds realTimePerTick((U32)(settings::dT * 1e6));
        //static const std::chrono::microseconds realTimePerTick(0);
    }
}

#include <sys/clock/Clock.hpp>

namespace sys {
    typedef clock::Clock Clock;
}  // namespace sys
