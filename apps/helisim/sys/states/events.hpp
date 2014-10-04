#pragma once
#ifndef SYS_STATES_EVENTS_HPP_
#define SYS_STATES_EVENTS_HPP_
#include <boost/statechart/event.hpp>

#define CREATE_EVENT(name)      struct name : boost::statechart::event< name > {}

namespace sys {
    namespace events {
        CREATE_EVENT(Initialized);
        CREATE_EVENT(StartSignal);
        CREATE_EVENT(GotGlobalPosition);
        CREATE_EVENT(FoundFire);
        CREATE_EVENT(ReachedObjective);
        CREATE_EVENT(OutOfObjectives);
        CREATE_EVENT(FinishedAction);
        CREATE_EVENT(FireExtinguished);
        CREATE_EVENT(Dying);

        CREATE_EVENT(FlippedUp);
        CREATE_EVENT(FlippedDown);
    }
}

#endif
