#pragma once
#ifndef SYS_SENSORHUB_SENSORS_GPS_HPP_
#define SYS_SENSORHUB_SENSORS_GPS_HPP_

#include <os/com/Dispatcher.hpp>
#include <os/drivers/Gps.hpp>
#include <sys/Sensorhub.hpp>

namespace sys {
    namespace sensorhub {
        class Gps {
            public:
                typedef Gps Self;
                Gps();
        };
    }
}

#endif
