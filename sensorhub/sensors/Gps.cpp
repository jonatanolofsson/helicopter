#include <sys/sensorhub/sensors/Gps.hpp>
#include <sys/math/models.hpp>
#include <sys/sensorhub/filtering.hpp>
#include <os/com/getSignal.hpp>

INSTANTIATE_SIGNAL(sys::sensorhub::sensors::Gps);

namespace sys {
    namespace sensorhub {
        Gps::Gps() : d(&Gps::handleMessage, this) {}

        void Gps::handleMessage(const SensorMessage) {
            typedef sys::sensorhub::sensors::Imu Measurement;
            Measurement m;

            yield(m);
        }
    }
}
