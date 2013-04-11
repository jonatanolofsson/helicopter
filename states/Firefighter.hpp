#pragma once
#ifndef SYS_STATES_FIREFIGHTER_HPP_
#define SYS_STATES_FIREFIGHTER_HPP_

#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/mpl/list.hpp>
#include <os/com/NetworkServer.hpp>
#include <os/com/TestMessages.hpp>

#include <sys/states/events.hpp>
#include <mutex>
#include <condition_variable>

namespace sys {
    namespace states {
        namespace mpl = boost::mpl;
        namespace sc = boost::statechart;

        struct Top;
        struct Firefighter : sc::state_machine<Firefighter, Top> {
            typedef mpl::list<
                sc::custom_reaction< events::Dying >
            > reactions;

            std::mutex m;
            std::condition_variable waitingForDeath;
            bool dying;
            Firefighter() : dying(false) {}
            void wait();
            void kill();
            void react(const events::Dying&);
        };
    }
}

#include <sys/states/Top.hpp>

#endif

