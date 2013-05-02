#include <sys/sensorhub/sensors/Gps.hpp>
#include <sys/math/models.hpp>
#include <sys/sensorhub/filtering.hpp>
#include <os/com/getSignal.hpp>

INSTANTIATE_SIGNAL(sys::sensorhub::sensors::Gps);

namespace sys {
    namespace sensorhub {
        Gps::Gps() : d(&Gps::handleMessage, this) {}

        void Gps::handleMessage(const GpsMessage) {
            typedef sys::sensorhub::sensors::Gps Measurement;
            Measurement m;

            yield(m);
        }
    }
}
