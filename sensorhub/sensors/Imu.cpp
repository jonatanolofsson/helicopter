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
            static const Scalar accelerometerScaling = settings::g / 16637.0;
            static const Scalar gyroscopeScaling = M_PI / (180.0 * 131.0);

            typedef sys::sensorhub::sensors::Imu Measurement;
            typedef Measurement::Sensor M;
            typedef ImuMessage R;
            Measurement m;
            m.z[M::a] = (float)(accelerometerScaling * msg.imu[R::ax]);
            //~ m.z[M::ay] = (float)(accelerometerScaling * msg.imu[R::ay]);
            //~ m.z[M::az] = (float)(accelerometerScaling * msg.imu[R::az]);
            //~ m.z[M::wx] = (float)(gyroscopeScaling * msg.imu[R::wx]);
            //~ m.z[M::wy] = (float)(gyroscopeScaling * msg.imu[R::wy]);
            m.z[M::w] = (float)(gyroscopeScaling * msg.imu[R::wz]);
            os::yield(m);

                 if(up && m.z[M::a] < 0) postEvent(events::FlippedDown());
            else if(!up && m.z[M::a] > 0) postEvent(events::FlippedUp());

            std::cout << (up ? "Up      : " : "Down    : ") << m.z.transpose() << std::endl;
        }
    }
}
