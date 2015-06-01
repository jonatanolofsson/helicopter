#pragma once

#include <boost/statechart/state.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/transition.hpp>

#include <sys/Clock.hpp>
#include <sys/Observer.hpp>
#include <sys/Sensorhub.hpp>
#include <sys/Logger.hpp>
#include <sys/com/Maple.hpp>
#include <sys/states/events.hpp>

#include <sys/StateMachine.hpp>
#include <os/utils/params.hpp>


namespace sys {
    namespace mpl = boost::mpl;
    namespace sc = boost::statechart;

    namespace states {
        struct Fly;
        struct Top : sc::simple_state<Top, statemachine::StateMachineEngine, Fly>
        {
            Observer::Filter globalFilter;
            Observer observer;

            Sensorhub sensorhub;
            Logger logger;

            Clock clock;

            Top()
            : observer(globalFilter)
            , logger("simlog.mat")
            , clock(os::parameters["Simulator"]["stoptime"].GetUint())
            {}
        };
    }
}

#include <sys/states/Fly.hpp>

