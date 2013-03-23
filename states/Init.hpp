#pragma once
#ifndef SYS_STATES_INIT_HPP_
#define SYS_STATES_INIT_HPP_
#include <sys/states/Top.hpp>

namespace sys {
    namespace states {
        struct Idle;
        struct ImuTest1;
        struct Init
        : sc::simple_state<Init, Top, ImuTest1>
        {
            typedef mpl::list<
                sc::transition< events::Initialized, Idle >
            > reactions;
        };
    }
}

#include <sys/states/ImuTest1.hpp>
#include <sys/states/ImuTest2.hpp>

#endif
