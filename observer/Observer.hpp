#pragma once
#ifndef SYS_OBSERVER_HPP_
#define SYS_OBSERVER_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/observer/API.hpp>

namespace sys {
    namespace observer {
        template<typename Filter, typename Model, typename FilterState, typename Trigger>
        class Observer {
            public:
                typedef Observer<Filter, Model, FilterState, Trigger> Self;
            private:
                os::Dispatcher<Self, Trigger> dispatcher;

                /* Sensors */
                //~ os::Dispatcher<Observer, sensors::Gps> gps;
                os::Dispatcher<Self, sensors::Imu> imu;

            public:
                FilterState state;

                Observer();

                void timeUpdate(const Trigger);

                template<typename Measurement, typename MUFilter = Filter>
                void measurementUpdate(const Measurement m) {
                    auto l = state.retrieve_lock();
                    MUFilter::template measurementUpdate<FilterState, Measurement>(state, m);
                }
        };
    }
}

#endif
