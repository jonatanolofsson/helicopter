#pragma once
#ifndef SYS_STATES_TOP_HPP_
#define SYS_STATES_TOP_HPP_

#include <boost/statechart/state.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/transition.hpp>

#include <sys/Clock.hpp>
#include <sys/Servomover.hpp>
#include <sys/com/Maple.hpp>
#include <sys/states/events.hpp>

#include <sys/states/Servotest.hpp>


namespace sys {
    namespace mpl = boost::mpl;
    namespace sc = boost::statechart;

    namespace states {
        struct MoveServos;
        struct Top : sc::simple_state<Top, Servotest>
        {
            Maple maple;
            Servomover servoMover;
            Top();
            ~Top();
            Clock clock;
        };
    }
}

#endif

