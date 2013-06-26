#include <sys/sensorhub/sensors/StmFFT.hpp>
#include <sys/math/filtering.hpp>
#include <sys/math/models.hpp>
#include <os/com/getSignal.hpp>
#include <sys/States.hpp>

#include <iostream>

namespace sys {
    namespace sensorhub {
        StmFFT::StmFFT() : d(&StmFFT::handleMessage, this) {}

        void StmFFT::handleMessage(const stm::FFTMessage) {
            postEvent(events::StartSignal());
        }
    }
}
