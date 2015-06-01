#pragma once
#ifndef SYS_STATEMACHINE_IMPLEMENTATION_HPP_
#define SYS_STATEMACHINE_IMPLEMENTATION_HPP_

#include <sys/statemachine/StateMachine.hpp>

namespace sys {
    namespace statemachine {
        template<typename InitialState>
        void StateEngine<InitialState>::kill() {
            std::unique_lock<std::mutex> l(m);
            dying = true;
            waitingForDeath.notify_all();
        }

        template<typename InitialState>
        void StateEngine<InitialState>::wait() {
            std::unique_lock<std::mutex> l(m);
            //LOG_EVENT(typeid(Self).name(), 0, "Waiting to die");
            while(!dying) {
                waitingForDeath.wait(l);
                //LOG_EVENT(typeid(Self).name(), 0, "Woke up to die");
            }
        }

        template<typename InitialState>
        void StateEngine<InitialState>::react(const events::Dying&) {
            kill();
        }

        template<typename InitialState>
        void StateMachine<InitialState>::postEvent(const boost::statechart::event_base& e) {
            stateMachine->process_event(e);
        }

        template<typename InitialState>
        void StateMachine<InitialState>::run() {
            StateEngine<InitialState> sm;
            stateMachine = &sm;
            stateMachine->initiate();
            stateMachine->wait();
        }

        template<typename InitialState>
        void StateMachine<InitialState>::kill() {
            stateMachine->kill();
        }
        template<typename InitialState>
        StateEngine<InitialState>* StateMachine<InitialState>::stateMachine;
    }
}

#endif
