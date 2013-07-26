#pragma once
#ifndef SYS_CLOCK_HPP_
#define SYS_CLOCK_HPP_

#include <thread>
#include <chrono>
#include <sys/Clock.hpp>
#include <os/clock.hpp>
#include <os/com/Dispatcher.hpp>

namespace sys {
    namespace clock {
        class Clock {
            public:
                typedef Clock Self;
            private:
                os::Dispatcher<Clock, os::Jiffy> d;
                void tick(const os::Jiffy);
                os::SystemTime time;
                SystemClock::time_point nextInvokation;

            public:
                Clock();
                ~Clock();
                void start();
                void stop();
        };
    }
}

#endif
