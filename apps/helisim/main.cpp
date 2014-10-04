#include <sys/States.hpp>
#include <os/com/getSignal.hpp>
#include <os/clock.hpp>

int main(int, char*[]) {
    sys::StateMachine stateMachine;
    sys::initStateMachine(stateMachine);
    sys::runStateMachine();

    return 0;
}
