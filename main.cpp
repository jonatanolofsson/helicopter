#include <sys/states/API.hpp>
#include <os/com/getSignal.hpp>
#include <os/clock.hpp>
#include <sys/global.hpp>

INSTANTIATE_SIGNAL(os::SystemTime);

namespace sys {
    StateMachine stateMachine;
    DebugServer debugServer;
}


int main(int, char*[]) {
    sys::runStateMachine();

    return 0;
}
