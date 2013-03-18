#pragma once
#ifndef SYS_CLOCK_HPP_
#define SYS_CLOCK_HPP_

#include <thread>
#include <chrono>
#include <sys/clock/API.hpp>
#include <os/clock.hpp>
#include <os/com/Dispatcher.hpp>

namespace sys {
    namespace clock {
        class Clock : os::Via<os::Jiffy> {
            private:
                os::Dispatcher<Clock, os::Jiffy> d;
                void tick(const os::Jiffy);
                os::SystemTime time;
                SystemClock::time_point nextInvokation;

            public:
                Clock();
        };
    }
}

#endif
