#pragma once
#ifndef SYS_COM_MAPLE_HPP_
#define SYS_COM_MAPLE_HPP_

#include <os/com/SerialCommunication.hpp>
#include <sys/com/MapleMessages.hpp>

namespace sys {
    typedef os::SerialCommunication<maple::Messages, 100, 100, B460800> Maple;
}

#endif
