#ifndef SYS_STATES_FLYING_TAKEOFF_HPP_
#define SYS_STATES_FLYING_TAKEOFF_HPP_

#include <boost/statechart.hpp>
#include <sys/states/Flying.hpp>

namespace sys {
    namespace sc = boost::statechart;

    struct TakeOff
    : sc::simple_state<TakeOff, Flying>
    {}
}

#endif
