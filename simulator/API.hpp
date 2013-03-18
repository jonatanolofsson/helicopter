#pragma once
#ifndef SYS_SIMULATOR_API_HPP_
#define SYS_SIMULATOR_API_HPP_

#include <sys/math/states.hpp>
#include <sys/math/models.hpp>
#include <sys/math/filtering.hpp>
#include <os/clock.hpp>

namespace sys {
    namespace simulator {
        typedef math::models::SCart3DAccQuat                States;
        typedef math::models::CVel3                         Controls;
        typedef math::models::Description<States, Controls> StateDescription;

        typedef models::motion::DirectVelocities3D          MotionModel;
        typedef math::GaussianFilter<StateDescription>      FilterState;

        namespace sensors {
            typedef models::sensors::Gps                    Gps;
            typedef models::sensors::Imu                    Imu;
        }
    }
}

#include <sys/simulator/Simulator.hpp>

namespace sys {
    typedef simulator::Simulator<simulator::MotionModel, simulator::FilterState> Simulator;
}

#endif
