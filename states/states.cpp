#include <sys/states/API.hpp>
#include <sys/states/Top.hpp>

namespace sys {
    namespace states {
        StateMachine stateMachine;

        void postEvent(const sc::event_base& e) {
            stateMachine.process_event(e);
        }

        void runStateMachine() {
            stateMachine.initiate();
            stateMachine.wait();
        }
    }
}
