#include <sys/sensorhub/sensors/Imu.hpp>
#include <sys/math/filtering.hpp>
#include <sys/math/models.hpp>
#include <os/com/getSignal.hpp>

INSTANTIATE_SIGNAL(sys::math::GaussianMeasurement<sys::models::sensors::Imu>);

namespace sys {
    namespace sensorhub {
        Imu::Imu() : d(&Imu::handleMessage, this) {}

        void Imu::handleMessage(const SensorMessage msg) {
            typedef math::GaussianMeasurement<models::sensors::Imu> Measurement;
            typedef Measurement::Model meas;
            typedef SensorMessage raw;
            Measurement m;
            m.z[meas::ax] = msg.imu[raw::ax];
            m.z[meas::ay] = msg.imu[raw::ax];
            m.z[meas::az] = msg.imu[raw::ax];
            yield(m);
        }
    }
}
