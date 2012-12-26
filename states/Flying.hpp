#ifndef SYS_STATES_FLYING_HPP_
#define SYS_STATES_FLYING_HPP_

#include <boost/statechart.hpp>
#include <sys/states/Top.hpp>

namespace sys {
    namespace sc = boost::statechart;

    struct TakeOff;

    struct Flying
    : sc::simple_state<Flying, Top, TakeOff>
    {
        MotionControl motionControl;
        Actuator actuator;
        Flying()
        : actuator(context<Top>().maple)
        {}
    }
}

#endif
