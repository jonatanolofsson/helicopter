#pragma once
#ifndef SYS_ACTUATOR_API_HPP_
#define SYS_ACTUATOR_API_HPP_

#include <sys/Motioncontrol.hpp>
#include <sys/com/Maple.hpp>

namespace sys {
    namespace actuator {
        typedef Maple SerialLink;

        typedef motioncontrol::ModelDescription         ModelDescription;
        typedef motioncontrol::Controls                 MotionControlSignal;

        static const Scalar wheelbase                   = 20.0;
    }
}

#include <sys/actuator/Actuator.hpp>

namespace sys {
    typedef actuator::Actuator<actuator::SerialLink>    Actuator;
}

#endif
