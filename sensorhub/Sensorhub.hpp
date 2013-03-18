#pragma once
#ifndef SYS_SENSORHUB_HPP_
#define SYS_SENSORHUB_HPP_

#include <sys/sensorhub/sensors/Imu.hpp>

namespace sys {
    namespace sensorhub {
        class Sensorhub {
            private:
                Imu imu;
        };
    }
}

#endif
