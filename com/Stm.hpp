#pragma once
#ifndef SYS_COM_STM_HPP_
#define SYS_COM_STM_HPP_

#include <os/com/SerialCommunication.hpp>
#include <sys/com/StmMessages.hpp>

namespace sys {
    typedef os::SerialCommunication<stm::Messages, 100, 10, B460800> Stm;
}

#endif
