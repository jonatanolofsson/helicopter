#pragma once
#ifndef SYS_SENSORHUB_API_HPP_
#define SYS_SENSORHUB_API_HPP_

#include <sys/Observer.hpp>
/*
 *#include <sys/sensorhub/sensors/Imu.hpp>
 */
#include <sys/sensorhub/sensors/Gps.hpp>

namespace sys {
    namespace sensorhub {
        struct Sensorhub {
            observer::sensors::Gps gps;
        };
    }
    
    typedef sensorhub::Sensorhub Sensorhub;
}

#endif
