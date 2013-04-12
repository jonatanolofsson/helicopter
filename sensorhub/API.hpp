#pragma once
#ifndef SYS_SENSORHUB_API_HPP_
#define SYS_SENSORHUB_API_HPP_

#include <sys/observer/API.hpp>

namespace sys {
    namespace sensorhub {
        namespace sensors {
            typedef observer::sensors::Imu                  Imu;
            typedef observer::sensors::Mouse                Mouse;
            typedef observer::sensors::ParticleFilterSensor ParticleFilterSensor;
            //~ typedef observer::sensors::Gps          Gps;
        }
    }
}

#include <sys/sensorhub/Sensorhub.hpp>

namespace sys {
    typedef sensorhub::Sensorhub Sensorhub;
}

#endif
