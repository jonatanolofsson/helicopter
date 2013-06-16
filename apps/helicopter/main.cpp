#include <sys/States.hpp>
#include <os/com/getSignal.hpp>
#include <os/clock.hpp>

namespace sys {
    StateMachine stateMachine;
}


int main(int, char*[]) {
    sys::runStateMachine();

    return 0;
}
