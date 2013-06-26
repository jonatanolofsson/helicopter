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
                typedef maple::SensorMessage ImuMessage;
                os::Dispatcher<Imu, ImuMessage> d;

            public:
                Imu();
                void handleMessage(const ImuMessage);

                // Remove after testing
                bool up;
        };
    }
}

#endif
