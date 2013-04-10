#pragma once
#ifndef SYS_ACTUATOR_API_HPP_
#define SYS_ACTUATOR_API_HPP_

#include <sys/actuator/Actuator.hpp>
#include <sys/com/Stm.hpp>
#include <sys/types.hpp>

namespace sys {
    namespace actuator {
        typedef Stm SerialLink;
        static const U16 firePermissionOffset   = 50;
        static const Scalar regP                = 10.0;
    }
    typedef actuator::Actuator<actuator::SerialLink> Actuator;
}

#endif
