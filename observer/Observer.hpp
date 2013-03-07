#ifndef OS_SYS_OBSERVER_HPP_
#define OS_SYS_OBSERVER_HPP_

#include <os/com/Dispatcher.hpp>

namespace sys {
    namespace observer {
        template<typename Filter, typename Model, typename FilterState, typename Trigger>
        class Observer {
            private:
                os::Dispatcher<Observer, Trigger> dispatcher;

            public:
                FilterState state;

                Observer() : dispatcher(&Observer::timeUpdate, this) {
                    FilterState::Model::StateDescription::initialize(state);
                }

                void timeUpdate(const Trigger) {
                    Filter::template timeUpdate<Model>(state, 1e-2);
                    os::yield(state.state);
                }

                template<typename Sensor, typename Measurement, typename MUFilter = Filter>
                void measurementUpdate(const Measurement m) {
                    MUFilter::template measurementUpdate<Model::States, Measurement>(state, m);
                }
        };
    }
}

#endif
