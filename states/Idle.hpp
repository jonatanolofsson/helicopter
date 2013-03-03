#ifndef SYS_STATES_IDLE_HPP_
#define SYS_STATES_IDLE_HPP_

#include <boost/statechart/simple_state.hpp>
#include <sys/states/Top.hpp>
#include <sys/states/OnTheGround.hpp>

namespace sys {
    namespace sc = boost::statechart;
    struct Idle
    : sc::simple_state<Idle, Top>
    {};
}

#endif
