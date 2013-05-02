#pragma once
#ifndef SYS_GLOBALPLANNER_HPP_
#define SYS_GLOBALPLANNER_HPP_

#include <os/com/Dispatcher.hpp>

namespace sys {
    namespace globalplanner {
        class GlobalPlanner {
            public:
                typedef GlobalPlanner Self;

            private:
                os::Dispatcher<Self, observer::SystemState> dispatcher;

            public:
                GlobalPlanner();
                void yieldReference(const observer::SystemState);
        }
    }
}

#endif
