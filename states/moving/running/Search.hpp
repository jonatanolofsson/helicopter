#pragma once
#ifndef SYS_STATES_RUNNING_SEARCH_HPP_
#define SYS_STATES_RUNNING_SEARCH_HPP_
#include <sys/states/moving/running/Running.hpp>

namespace sys {
    namespace states {
        struct Extinguish;
        struct Search
        : sc::simple_state<Search, Running>
        {
            typedef mpl::list<
                sc::transition< events::FoundFire, Extinguish >,
                sc::custom_reaction< events::ReachedObjective >
            > reactions;

            Search();

            sc::result react(const events::ReachedObjective&);
        };
    }
}

#endif
