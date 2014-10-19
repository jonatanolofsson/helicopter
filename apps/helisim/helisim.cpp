#include <sys/StateMachine.hpp>
#include <os/com/getSignal.hpp>
#include <os/clock.hpp>

int main(int, char*[]) {
    sys::StateMachine::run();

    return 0;
}
