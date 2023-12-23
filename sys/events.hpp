#pragma once
#include <boost/statechart/event.hpp>

#define CREATE_EVENT(name)      struct name : boost::statechart::event< name > {}

namespace sys {
    namespace events {
        CREATE_EVENT(Dying);
    }
}
