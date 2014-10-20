#pragma once
#ifndef SYS_LOGGER_API_HPP_
#define SYS_LOGGER_API_HPP_

#include <sys/logger/Logger.hpp>
#include <os/clock.hpp>
#include <sys/Observer.hpp>
#include <sys/MotionControl.hpp>

namespace sys {
#define LOGGERCLASS logger::Logger<os::SystemTime, Observer::StateMessage, motioncontrol::Reference, motioncontrol::ControlMessage>
    typedef LOGGERCLASS Logger;
}

#endif
