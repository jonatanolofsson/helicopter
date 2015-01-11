#include <sys/StateMachine.hpp>
#include <os/com/getSignal.hpp>
#include <os/clock.hpp>
#include <os/core/init.hpp>

int main(int argc, char** argv) {
    os::init(argc, argv);
    sys::StateMachine::run();

    std::cout << "Exiting" << std::endl;

    return 0;
}
