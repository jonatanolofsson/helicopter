#pragma once
#ifndef SYS_SENSORHUB_HPP_
#define SYS_SENSORHUB_HPP_

#include <sys/sensorhub/sensors/Imu.hpp>
#include <sys/sensorhub/sensors/StmFFT.hpp>
#include <sys/sensorhub/sensors/StmIR.hpp>
#include <sys/sensorhub/sensors/Mouse.hpp>

namespace sys {
    namespace sensorhub {
        struct Sensorhub {
            Imu imu;
            StmIR stmIr;
            StmFFT stmFFT;
            Mouse mouse;
        };
    }
}

#endif
