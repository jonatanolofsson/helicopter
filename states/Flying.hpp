#ifndef SYS_STATES_FLYING_HPP_
#define SYS_STATES_FLYING_HPP_

#include <boost/statechart.hpp>
#include <sys/states/Top.hpp>
#include <sys/motioncontrol/API.hpp>
#include <sys/actuator/API.hpp>

namespace sys {
    namespace sc = boost::statechart;
    using actuator::Actuator;
    using motioncontrol::MotionControl;

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
