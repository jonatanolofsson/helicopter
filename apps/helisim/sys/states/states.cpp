#include <sys/States.hpp>
#include <sys/states/Top.hpp>

namespace sys {
    StateMachine* stateMachine;

    void initStateMachine(StateMachine& sm) {
        stateMachine = &sm;
    }

    void postEvent(const sc::event_base& e) {
        stateMachine->process_event(e);
    }

    void runStateMachine() {
        stateMachine->initiate();
        stateMachine->wait();
    }

    void killStateMachine() {
        stateMachine->kill();
    }
}
