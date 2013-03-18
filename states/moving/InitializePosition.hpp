#pragma once
#ifndef SYS_STATES_INITIALIZEPOSITION_HPP_
#define SYS_STATES_INITIALIZEPOSITION_HPP_
#include <sys/states/moving/Moving.hpp>

namespace sys {
    namespace states {
        struct Running;
        struct InitializePosition
        : sc::simple_state<InitializePosition, Moving>
        {
            typedef mpl::list<
                sc::transition< events::GotGlobalPosition, Running >
            > reactions;

            InitializePosition();
        };
    }
}

#endif
