#include <sys/sensorhub/sensors/Imu.hpp>
#include <sys/math/filtering.hpp>
#include <sys/math/models.hpp>
#include <os/com/getSignal.hpp>
#include <sys/states/API.hpp>

#include <iostream>

INSTANTIATE_SIGNAL(sys::sensorhub::sensors::Imu);

namespace sys {
    namespace sensorhub {
        Imu::Imu() : d(&Imu::handleMessage, this) {}

        void Imu::handleMessage(const ImuMessage msg) {
            typedef sys::sensorhub::sensors::Imu Measurement;
            typedef Measurement::Sensor M;
            typedef ImuMessage R;
            Measurement m;
            m.z[M::ax] = (float)msg.imu[R::ax];
            m.z[M::ay] = (float)msg.imu[R::ay];
            m.z[M::az] = (float)msg.imu[R::az];
            m.z[M::wx] = (float)msg.imu[R::wx];
            m.z[M::wy] = (float)msg.imu[R::wy];
            m.z[M::wz] = (float)msg.imu[R::wz];
            os::yield(m);

                 if(up && m.z[M::ax] < 0) postEvent(events::FlippedDown());
            else if(!up && m.z[M::ax] > 0) postEvent(events::FlippedUp());

            std::cout << (up ? "Up      : " : "Down    : ") << m.z.transpose() << std::endl;
        }
    }
}
