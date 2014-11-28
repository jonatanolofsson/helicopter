#pragma once

#include <sys/math/states.hpp>
#include <sys/math/models.hpp>
#include <sys/simulator/Simulator.hpp>
#include <sys/MotionControl.hpp>
#include <sys/Observer.hpp>

namespace sys {
    namespace simulator {
        typedef math::models::HelicopterStates States;
        typedef math::models::HelicopterMotion<States> MotionModel;
        typedef motioncontrol::ControlMessage ControlMessage;
        typedef Observer::Filter GlobalFilter;
        typedef SimulatedStateMessage<States> SimulatedStateMessage;
    }
#define SIMULATOR_CLASS simulator::Simulator<simulator::GlobalFilter, simulator::MotionModel, simulator::ControlMessage, observer::sensors::Gps, observer::sensors::Imu>
    typedef SIMULATOR_CLASS Simulator;
}

