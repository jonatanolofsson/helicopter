#include <sys/states/Top.hpp>
#include <os/com/getSignal.hpp>
#include <os/clock.hpp>

INSTANTIATE_SIGNAL(os::SystemTime);

using namespace sys;

Firefighter stateMachine;

int main(int, char*[]) {
    stateMachine.initiate();
    while(true) sleep(1000);

    return 0;
}
