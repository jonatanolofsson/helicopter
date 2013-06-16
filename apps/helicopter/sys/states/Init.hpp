#pragma once
#ifndef SYS_STATES_INIT_HPP_
#define SYS_STATES_INIT_HPP_
#include <sys/states/Top.hpp>
#include <sys/states/Idle.hpp>

namespace sys {
    namespace states {
        struct Idle;
        struct Init
        : sc::simple_state<Init, Top>
        {
            typedef mpl::list<
                sc::transition< events::Initialized, Idle >
            > reactions;
        };
    }
}

#endif
