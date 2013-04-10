#pragma once
#ifndef SYS_STATES_MOVING_HPP_
#define SYS_STATES_MOVING_HPP_
#include <sys/states/Top.hpp>
#include <sys/actuator/API.hpp>
#include <sys/localplanner/API.hpp>
#include <sys/particlefilter/API.hpp>

namespace sys {
    namespace states {
        struct InitializePosition;
        struct Moving
        : sc::state<Moving, Top, InitializePosition>
        {
            Actuator actuator;
            LocalPlanner planner;
            ParticleFilter particleFilter;
            Moving(my_context ctx);

            const localplanner::Checkpoint* startingPosition;
        };
    }
}

#include <sys/states/moving/InitializePosition.hpp>
#include <sys/states/moving/running/Running.hpp>

#endif
