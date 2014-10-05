#include <sys/sensorhub/sensors/Gps.hpp>
#include <sys/math/models.hpp>
#include <os/com/getSignal.hpp>
#include <gps.h>

/*
 *INSTANTIATE_SIGNAL(sys::sensorhub::sensors::Gps);
 */

namespace sys {
    namespace sensorhub {
        Gps::Gps() {
            /*
             *gps_open("localhost", DEFAULT_GPSD_PORT)
             */
        }

/*
 *        void Gps::handleMessage(const GpsMessage) {
 *            typedef sys::sensorhub::sensors::Gps Measurement;
 *            Measurement m;
 *
 *            yield(m);
 *        }
 */
    }
}
