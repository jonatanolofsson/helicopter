#pragma once

#include <boost/statechart/state.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/transition.hpp>

#include <sys/com/Maple.hpp>
#include <sys/StateMachine.hpp>

#include "sys/Clock.hpp"
#include "sys/Servomover.hpp"
#include "sys/states/events.hpp"


namespace sys {
    namespace mpl = boost::mpl;
    namespace sc = boost::statechart;

    namespace states {
        struct Top : sc::simple_state<Top, statemachine::StateMachineEngine>
        {
            Maple maple;
            Servomover servoMover;
            Top();
            ~Top();
            Clock clock;
        };
    }
}
