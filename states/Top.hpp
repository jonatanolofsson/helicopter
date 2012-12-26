#ifndef SYS_STATES_TOP_HPP_
#define SYS_STATES_TOP_HPP_

#include <boost/statechart.hpp>
#include <MotionControl/MotionControl.hpp>

namespace sys {
    namespace sc = boost::statechart;

    struct Top;
    struct Idle;

    struct Helicopter : sc::state_machine<Helicopter, Top>
    {}

    struct Top : sc::simple_state<Top, Helicopter, Idle>
    {
        Observer observer;
        NetworkServer<ImageProcessorMessages, 100, 10> imageProcessor;
        SerialCommunication<ImageProcessorMessages, 100, 10> maple;
    }
}


#endif

