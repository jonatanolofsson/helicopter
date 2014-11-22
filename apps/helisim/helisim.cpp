#include <sys/StateMachine.hpp>
#include <os/com/getSignal.hpp>
#include <os/clock.hpp>

int main(int, char*[]) {
    sys::StateMachine::run();

    std::cout << "Exiting" << std::endl;

    return 0;
}
