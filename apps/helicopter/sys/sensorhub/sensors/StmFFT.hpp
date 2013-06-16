#pragma once
#ifndef SYS_SENSORHUB_SENSORS_STMFFT_HPP_
#define SYS_SENSORHUB_SENSORS_STMFFT_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/com/Stm.hpp>

#include <string>

namespace sys {
    namespace sensorhub {
        class StmFFT {
            private:
                os::Dispatcher<StmFFT, stm::FFTMessage> d;

            public:
                StmFFT();
                void handleMessage(const stm::FFTMessage);

                // Remove after testing
                bool up;
        };
    }
}

#endif
