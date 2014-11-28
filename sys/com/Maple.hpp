#pragma once

#include <os/com/SerialCommunication.hpp>
#include <sys/com/MapleMessages.hpp>

namespace sys {
    typedef os::SerialCommunication<maple::Messages, 100, 100, B460800> Maple;
}

