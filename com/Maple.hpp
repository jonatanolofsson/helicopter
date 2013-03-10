#ifndef SYS_COM_MAPLE_HPP_
#define SYS_COM_MAPLE_HPP_

#include <os/com/SerialCommunication.hpp>
#include <sys/com/MapleMessages.hpp>

namespace sys {
    typedef os::SerialCommunication<MapleMessages, 100, 10, B115200> Maple;
}

#endif
