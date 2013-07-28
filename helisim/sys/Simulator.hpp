#pragma once
#ifndef SYS_SIMULATOR_API_HPP_
#define SYS_SIMULATOR_API_HPP_

#include <sys/math/states.hpp>
#include <sys/math/models.hpp>
#include <os/clock.hpp>
#include <sys/simulator/Simulator.hpp>
#include <sys/Observer.hpp>

namespace sys {
    namespace simulator {
        typedef math::models::SCart3D                       StateDescription;
        typedef math::models::CVel3                         ControlDescription;
        typedef math::models::Description<StateDescription, ControlDescription> ModelDescription;

        typedef math::models::DirectVelocities3D<ModelDescription> MotionModel;
    }
    typedef simulator::Simulator<simulator::MotionModel, observer::sensors::Gps> Simulator;
}

#endif
