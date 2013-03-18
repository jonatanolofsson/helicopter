#pragma once
#ifndef SYS_STATES_STOP_HPP_
#define SYS_STATES_STOP_HPP_
#include <sys/states/Top.hpp>

namespace sys {
    namespace states {
        struct Stop
        : sc::simple_state<Stop, Top>
        {};
    }
}

#endif
