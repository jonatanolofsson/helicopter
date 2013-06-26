#pragma once
#ifndef SYS_OBSERVER_HPP_
#define SYS_OBSERVER_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/Servomover.hpp>
#include <type_traits>

namespace sys {
    namespace servomover {
        template<typename Trigger, typename Serial>
        class Servomover {
            public:
                typedef Servomover<Trigger, Serial> Self;
            private:
                Serial& maple;
                os::Dispatcher<Self, Trigger> dispatcher;
                typename Serial::Messages::ControlMessage ctrlMsg;
            public:
                Servomover(Serial& maple);

                void servoUpdate(const Trigger);
        };
    }
}

#endif
