#pragma once
#ifndef SYS_STATES_HPP_
#define SYS_STATES_HPP_

#include <boost/statechart/event.hpp>
#include <sys/states/events.hpp>

namespace sys {
    namespace states {
        namespace sc = boost::statechart;

        void postEvent(const sc::event_base&);
        void runStateMachine();
    }

    using states::postEvent;
    using states::runStateMachine;
}

#include <sys/states/Firefighter.hpp>

#endif
