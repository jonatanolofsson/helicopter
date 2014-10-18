#pragma once
#ifndef SYS_COM_STATEMESSAGE_HPP_
#define SYS_COM_STATEMESSAGE_HPP_

#include <sys/types.hpp>
#include <sys/math/filtering.hpp>
#include <sys/com/eigenmessage.hpp>

namespace sys {
    namespace math {
        namespace models {
            namespace messages {
                template<typename States> struct StateMessage : public EigenMessage<States> { StateMessage(){} explicit StateMessage(const States& s) : EigenMessage<States>(s) {} };
            }
        }
    }
}

#endif

