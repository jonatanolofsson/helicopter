#include <boost/statechart/event.hpp>

#define CREATE_EVENT(name)      struct name : sc::event< name > {}

namespace sys {
    namespace events {
        namespace sc = boost::statechart;

        CREATE_EVENT(Initialized);
        CREATE_EVENT(StartSignal);
        CREATE_EVENT(GotGlobalPosition);
        CREATE_EVENT(FoundFire);
        CREATE_EVENT(ReachedObjective);
        CREATE_EVENT(FinishedAction);
        CREATE_EVENT(FireExtinguished);
    }
}
