#pragma once
#ifndef SYS_SENSORHUB_SENSORS_MOUSE_HPP_
#define SYS_SENSORHUB_SENSORS_MOUSE_HPP_

#include <os/com/Dispatcher.hpp>
#include <mutex>

namespace sys {
    namespace sensorhub {
        class Mouse {
            private:
                os::Dispatcher<Mouse, os::SystemTime> dispatcher;
                std::thread readerThread;
                void timelyReader(const os::SystemTime);
                void mouseReader();
                bool dying;
                U32 dots;
                std::mutex inputGuard;

            public:
                Mouse();
                ~Mouse();
        };
    }
}

#endif
