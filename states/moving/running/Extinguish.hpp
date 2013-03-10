#ifndef SYS_STATES_RUNNING_EXTINGUISH_HPP_
#define SYS_STATES_RUNNING_EXTINGUISH_HPP_
#include <sys/states/moving/running/Running.hpp>

namespace sys {
    namespace states {
        struct Return;
        struct Extinguish
        : sc::simple_state<Extinguish, Running>
        {
            typedef mpl::list<
                sc::transition< events::FireExtinguished, Return >
            > reactions;

            Extinguish();
            ~Extinguish();
        };
    }
}

#endif
