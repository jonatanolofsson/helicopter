#ifndef SYS_STATES_RUNNING_HPP_
#define SYS_STATES_RUNNING_HPP_
#include <sys/states/moving/Moving.hpp>
#include <sys/motioncontrol/API.hpp>
#include <sys/localplanner/API.hpp>

namespace sys {
    namespace states {
        struct Search;
        struct Running
        : sc::simple_state<Running, Moving, Search>
        {
            MotionControl motionControl;
            LocalPlanner localPlanner;

            Running();
        };
    }
}

#include <sys/states/moving/running/Search.hpp>
#include <sys/states/moving/running/Extinguish.hpp>
#include <sys/states/moving/running/Return.hpp>

#endif
