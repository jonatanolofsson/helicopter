#ifndef SYS_STATES_IDLE_HPP_
#define SYS_STATES_IDLE_HPP_
#include <sys/states/Top.hpp>

namespace sys {
    namespace states {
        struct Moving;
        struct Idle
        : sc::simple_state<Idle, Top>
        {
            typedef mpl::list<
                sc::transition< events::StartSignal, Moving >
            > reactions;
        };
    }
}

#endif
