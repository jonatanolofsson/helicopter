#include <sys/sensorhub/sensors/Gps.hpp>
#include <sys/math/models.hpp>
#include <sys/sensorhub/filtering.hpp>
#include <os/com/getSignal.hpp>

INSTANTIATE_SIGNAL(sys::math::GaussianMeasurement<models::Gps>);

namespace sys {
    namespace sensorhub {
        Gps::Gps() : d(&Gps::handleMessage, this) {}

        void Gps::handleMessage(const SensorMessage) {
            typedef math::GaussianMeasurement<models::Gps> Measurement;
            Measurement m;

            yield(m);
        }
    }
}
