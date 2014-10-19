#pragma once
#ifndef REFERENCEGENERATOR_HPP_
#define REFERENCEGENERATOR_HPP_

#include <sys/types.hpp>
#include <sys/referencegenerator/ReferenceGenerator.hpp>
#include <sys/MotionControl.hpp>
#include <sys/com/statemessage.hpp>

namespace sys {
    namespace referencegenerator {
        typedef ReferenceMessage<motioncontrol::States> ReferenceMessage;
        typedef os::SystemTime Trigger;
    }

    typedef referencegenerator::ReferenceGenerator<referencegenerator::ReferenceMessage, referencegenerator::Trigger> ReferenceGenerator;
}

#endif
