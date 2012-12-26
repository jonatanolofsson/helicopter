#ifndef AC_SERIAL_HPP_
#define AC_SERIAL_HPP_

#include <sys/com/MapleMessages.hpp>
#include <os/com/IoCommunication.hpp>
#include <syrup/comm/ByteInterface.hpp>
#include <syrup/comm/SerialPort.hpp>
#include <Wire/HardWire.h>

using namespace os;
using namespace syrup;

template<typename M, int MAX_MESSAGE_SIZE, int BAUD_RATE = 115200, typename T = HardwareSerial>
class SerialCommunication : public IoCommunication<M, MAX_MESSAGE_SIZE> {
    public:
        typedef IoCommunication<M, MAX_MESSAGE_SIZE> Parent;
        using Parent::socket;
        SerialPort<T> device;

    public:
        SerialCommunication(T* dev)
        : Parent()
        , device(dev)
        {
            dev->begin(BAUD_RATE);
            socket = open(&device);
            Parent::start();
        }
};

typedef SerialCommunication<sys::MapleMessages, 50, 38400> ComputerLink;

#endif
