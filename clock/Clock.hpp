#ifndef SYS_CLOCK_HPP_
#define SYS_CLOCK_HPP_

#include <thread>
#include <os/clock.hpp>
#include <os/com/Dispatcher.hpp>

namespace sys {
    namespace clock {
        class Clock : os::Via<os::Jiffy> {
            private:
                std::thread t;
                void tick(const os::Jiffy);
                void run();
                os::SystemTime time;
                bool dying;

            public:
                Clock();
                ~Clock();
                void start();
                void stop();
        };
    }
}

#endif
