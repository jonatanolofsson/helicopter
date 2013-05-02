#pragma once
#ifndef SYS_GLOBAL_HPP_
#define SYS_GLOBAL_HPP_

#include <os/com/NetworkServer.hpp>
#include <os/com/TestMessages.hpp>

namespace sys {
    typedef os::NetworkServer<50007, os::testmessages::Messages, 100, 10> DebugServer;

    extern DebugServer debugServer;
}

#endif
