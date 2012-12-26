#ifndef SYS_STATES_ONTHEGROUND_HPP_
#define SYS_STATES_ONTHEGROUND_HPP_

#include <boost/statechart.hpp>
#include <sys/states/Top.hpp>

namespace sys {
    namespace sc = boost::statechart;
    struct OnTheGround
    : sc::simple_state<OnTheGround, Top>
    {}
}

#endif
