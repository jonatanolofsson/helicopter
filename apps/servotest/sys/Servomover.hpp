#pragma once
#ifndef SYS_OBSERVER_API_HPP_
#define SYS_OBSERVER_API_HPP_

#include <os/clock.hpp>
#include <sys/com/Maple.hpp>

namespace sys {
    namespace servomover {
        typedef os::SystemTime                              TriggerType;
        typedef Maple SerialLink;
    }
}

#include <sys/servomover/Servomover.hpp>

namespace sys {
    typedef servomover::Servomover<servomover::TriggerType, servomover::SerialLink> Servomover;
}

#endif
