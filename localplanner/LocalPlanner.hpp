#pragma once
#ifndef SYS_LOCALPLANNER_HPP_
#define SYS_LOCALPLANNER_HPP_

#include <sys/localplanner/API.hpp>
#include <sys/observer/API.hpp>
#include <os/com/Dispatcher.hpp>
#include <sys/types.hpp>
#include <sys/math/algorithm.hpp>

namespace sys {
    namespace localplanner {
        class LocalPlanner {
            public:
                typedef LocalPlanner Self;

            private:
                std::list<const Checkpoint*> objectives;
                std::stack<const Checkpoint*> checkpoints;
                os::Dispatcher<Self, observer::SystemState> d_reference;
                const Checkpoint* closestCheckpoint;

            public:
                LocalPlanner();
                void yieldReference(const observer::SystemState);
                void clearObjectives();
                void addObjective(const Checkpoint*);
                const Checkpoint* getClosestCheckpoint();
        };
    }
}

#endif
