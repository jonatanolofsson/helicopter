#include <sys/states/API.hpp>
#include <os/com/getSignal.hpp>
#include <os/clock.hpp>

INSTANTIATE_SIGNAL(os::SystemTime);

using namespace sys;


int main(int, char*[]) {
    runStateMachine();

    return 0;
}
