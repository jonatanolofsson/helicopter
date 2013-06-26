#pragma once
#ifndef SYS_EVENTS_HPP_
#define SYS_EVENTS_HPP_
#include <boost/statechart/event.hpp>

#define CREATE_EVENT(name)      struct name : boost::statechart::event< name > {}

namespace sys {
    namespace events {
        CREATE_EVENT(Dying);
    }
}

#endif
