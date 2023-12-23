#include <os/core/init.hpp>
#include <sys/StateMachine.hpp>


int main(int argc, char** argv) {
    os::init(argc, argv);
    sys::StateMachine::run();

    return 0;
}
