#pragma once
#ifndef SYS_STATES_MOVING_HPP_
#define SYS_STATES_MOVING_HPP_
#include <sys/states/Top.hpp>
#include <sys/actuator/API.hpp>

namespace sys {
    namespace states {
        struct InitializePosition;
        struct Moving
        : sc::simple_state<Moving, Top, InitializePosition>
        {
            Actuator actuator;
            Moving();
        };
    }
}

#include <sys/states/moving/InitializePosition.hpp>
#include <sys/states/moving/running/Running.hpp>

#endif
