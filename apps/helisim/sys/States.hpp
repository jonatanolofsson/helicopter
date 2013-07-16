#pragma once
#ifndef SYS_STATES_HPP_
#define SYS_STATES_HPP_

#include <boost/statechart/event.hpp>
#include <sys/states/events.hpp>

namespace sys {
    void postEvent(const boost::statechart::event_base&);
    void runStateMachine();
}

#include <sys/states/Helicopter.hpp>

namespace sys {
    typedef states::Helicopter StateMachine;
    void initStateMachine(StateMachine& stateMachine);
    void killStateMachine();
}

#endif
