#pragma once
#ifndef SYS_STATES_TOP_HPP_
#define SYS_STATES_TOP_HPP_

#include <boost/statechart/state.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/transition.hpp>

#include <sys/Clock.hpp>
#include <sys/Observer.hpp>
#include <sys/Sensorhub.hpp>
#include <sys/Logger.hpp>
#include <sys/com/Maple.hpp>
#include <sys/states/events.hpp>

#include <sys/states/Helicopter.hpp>


namespace sys {
    namespace mpl = boost::mpl;
    namespace sc = boost::statechart;

    namespace states {
        struct Fly;
        struct Top : sc::simple_state<Top, Helicopter, Fly>
        {
            Observer observer;

            Sensorhub sensorhub;
            Logger logger;

            Clock clock;

            Top() : logger("simlog.mat") {}
        };
    }
}

#include <sys/states/Fly.hpp>

#endif

