#pragma once
#ifndef SYS_SIMULATOR_API_HPP_
#define SYS_SIMULATOR_API_HPP_

#include <sys/math/states.hpp>
#include <sys/math/models.hpp>
#include <sys/simulator/Simulator.hpp>
#include <sys/MotionControl.hpp>

namespace sys {
    namespace simulator {
        typedef math::models::VWXQ_3D States;
        typedef math::models::Velocity_XQ_3D<States> MotionModel;
        typedef motioncontrol::ControlMessage ControlMessage;
    }
    typedef simulator::Simulator<simulator::MotionModel, simulator::ControlMessage, observer::sensors::Gps> Simulator;
}

#endif
