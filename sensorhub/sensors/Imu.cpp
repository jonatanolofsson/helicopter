#include <sys/sensorhub/sensors/Imu.hpp>
#include <sys/math/filtering.hpp>
#include <sys/math/models.hpp>
#include <os/com/getSignal.hpp>
#include <sys/states/API.hpp>

#include <iostream>

INSTANTIATE_SIGNAL(sys::math::GaussianMeasurement<sys::models::sensors::Imu>);

namespace sys {
    namespace sensorhub {
        Imu::Imu() : d(&Imu::handleMessage, this) {}

        void Imu::handleMessage(const SensorMessage msg) {
            typedef math::GaussianMeasurement<models::sensors::Imu> Measurement;
            typedef Measurement::Model meas;
            typedef SensorMessage raw;
            Measurement m;
            m.z[meas::ax] = (S16)msg.imu[raw::ax];
            m.z[meas::ay] = (S16)msg.imu[raw::ax];
            m.z[meas::az] = (S16)msg.imu[raw::ax];
            m.z[meas::wx] = (S16)msg.imu[raw::wx];
            m.z[meas::wy] = (S16)msg.imu[raw::wx];
            m.z[meas::wz] = (S16)msg.imu[raw::wx];
            //~ yield(m);

                 if(up && m.z[meas::ax] < 0) postEvent(events::FlippedDown());
            else if(!up && m.z[meas::ax] > 0) postEvent(events::FlippedUp());

            std::cout << (up ? "Up      : " : "Down    : ") << m.z.transpose() << std::endl;
        }
    }
}
