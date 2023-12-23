#include <sys/com/Maple.hpp>
#include <sys/types.hpp>
#include <iostream>

int main(int argn, char* argv[]) {
    using namespace sys;
    Maple maple("/dev/ttyUSB0");
    if (argn == 3) {
        U16 whichServo = *argv[1] - '1';
        U16 howMuch = atoi(argv[2]);
        if (whichServo < 4) {
            Maple::Messages::ControlMessage ctrlMsg = {};
            if (whichServo < 3) {
                ctrlMsg.servo[whichServo] = howMuch;
            } else {
                ctrlMsg.rpm = howMuch;
            }

            std::cout << "Sending " << howMuch
                << " to servo " << whichServo << std::endl;
            maple.send(ctrlMsg);
            maple.wait();
            return 0;
        }
    }

    std::cerr << "The command accepts two, numeric arguments" << std::endl;
    return -1;
}
