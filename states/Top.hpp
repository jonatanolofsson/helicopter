#ifndef SYS_STATES_TOP_HPP_
#define SYS_STATES_TOP_HPP_

#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <sys/MotionControl.hpp>
#include <sys/com/MapleMessages.hpp>
#include <sys/com/ImageProcessorMessages.hpp>
#include <sys/Observer/Observer.hpp>
#include <os/com/NetworkServer.hpp>
#include <os/com/SerialCommunication.hpp>

namespace sys {
    namespace sc = boost::statechart;

    struct Top;
    struct Idle;

    struct Helicopter : sc::state_machine<Helicopter, Top>
    {};

    struct Top : sc::simple_state<Top, Helicopter, Idle>
    {
        Observer::Observer observer;
        NetworkServer<8810, ImageProcessorMessages, 100, 10> imageProcessor;
        SerialCommunication<MapleMessages, 100, 10> maple;

        Top();
    };
}

#include <sys/states/Idle.hpp>

#endif

