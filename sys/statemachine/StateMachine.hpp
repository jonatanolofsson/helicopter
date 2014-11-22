#pragma once
#ifndef SYS_STATEMACHINE_HPP_
#define SYS_STATEMACHINE_HPP_

#include <mutex>
#include <condition_variable>

#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/mpl/list.hpp>
#include <boost/statechart/event.hpp>

#include <sys/events.hpp>

namespace sys {
    namespace statemachine {
        namespace mpl = boost::mpl;
        namespace sc = boost::statechart;

        template<typename InitialState>
        class StateEngine : public sc::state_machine<StateEngine<InitialState>, InitialState> {
            private:
                typedef StateEngine<InitialState> Self;

                std::mutex m;
                std::condition_variable waitingForDeath;
                volatile bool dying;
            public:
                StateEngine() : dying(false) {}

                typedef mpl::list<
                    sc::custom_reaction< events::Dying >
                > reactions;

                void kill();
                void wait();
                void react(const events::Dying&);
        };

        template<typename InitialState>
        class StateMachine {
            private:
                static StateEngine<InitialState>* stateMachine;

            public:
                static void postEvent(const boost::statechart::event_base&);
                static void run();
                static void kill();
        };
    }
}

#endif
