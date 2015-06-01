#include <sys/Clock.hpp>
#include <ctime>
#include <chrono>
#include <os/com/Dispatcher.hpp>
#include <os/utils/eventlog.hpp>
#include <iostream>

#include <sys/StateMachine.hpp>

namespace sys {
    namespace clock {
        Clock::Clock(const unsigned stoptime_)
        : d(&Clock::tick, this)
        , stoptime(stoptime_)
        , nextInvokation(SystemClock::now())
        {}

        void Clock::start() {
            os::startTime();
        }

        void Clock::stop() {
            os::stopTime();
        }

        Clock::~Clock() {
            //LOG_EVENT(typeid(Self).name(), 0, "Stopping time");
            stop();
        }

        void Clock::setStopTime(const unsigned stoptime_) {
            stoptime = stoptime_;
            if(stoptime && (time.value >= stoptime)) {
                stop();
                StateMachine::kill();
            }
        }

        void Clock::tick(const os::Jiffy) {
            if(stoptime && (time.value >= stoptime)) {
                stop();
                StateMachine::kill();
                return;
            }
            std::this_thread::sleep_until(nextInvokation);
            os::yield(++time);
            /*
             *LOG_EVENT(typeid(Self).name(), 0, "Upped time: " << time << "(" << j.value << ")");
             */
            nextInvokation = SystemClock::now() + realTimePerTick;
        }
    }
}
