#ifndef SYS_LOCALPLANNER_HPP_
#define SYS_LOCALPLANNER_HPP_

#include <sys/localplanner/API.hpp>
#include <sys/observer/API.hpp>
#include <os/com/Dispatcher.hpp>
#include <sys/types.hpp>
#include <mutex>

namespace sys {
    namespace localplanner {

        struct Checkpoint {
            Scalar x, y;
        };

        class LocalPlanner {
            public:
                typedef LocalPlanner Self;

            private:
                Checkpoint destination;
                std::mutex guard;
                os::Dispatcher<Self, observer::SystemState> d_reference;
                os::Dispatcher<Self, Checkpoint> d_destination;

            public:
                LocalPlanner();
                void yieldReference(const observer::SystemState);
                void updateDestination(const Checkpoint);
        };
    }
}

#endif
