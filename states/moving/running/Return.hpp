#ifndef SYS_STATES_RUNNING_RETURN_HPP_
#define SYS_STATES_RUNNING_RETURN_HPP_
#include <sys/states/moving/running/Running.hpp>

namespace sys {
    namespace states {
        struct Stop;
        struct Return
        : sc::simple_state<Return, Running>
        {
            typedef mpl::list<
                sc::transition< events::ReachedObjective, Stop >
            > reactions;

            Return();
        };
    }
}

#endif
