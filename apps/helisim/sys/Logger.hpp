#pragma once

#include <sys/logger/Logger.hpp>
#include <os/clock.hpp>
#include <sys/Observer.hpp>
#include <sys/MotionControl.hpp>
#include <sys/Simulator.hpp>

namespace sys {
#define LOGGERCLASS logger::Logger<os::SystemTime, Observer::StateMessage, motioncontrol::Reference, motioncontrol::ControlMessage, simulator::SimulatedStateMessage>
    typedef LOGGERCLASS Logger;
}

