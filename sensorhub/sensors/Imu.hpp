#pragma once
#ifndef SYS_SENSORHUB_SENSORS_IMU_HPP_
#define SYS_SENSORHUB_SENSORS_IMU_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/com/Maple.hpp>

#include <string>

namespace sys {
    namespace sensorhub {
        class Imu {
            private:
                os::Dispatcher<Imu, SensorMessage> d;

            public:
                Imu();
                void handleMessage(const SensorMessage);

                // Remove after testing
                bool up;
        };
    }
}

#endif
