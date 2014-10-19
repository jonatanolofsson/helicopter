#pragma once
#ifndef SYS_OBSERVER_HPP_
#define SYS_OBSERVER_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/Observer.hpp>
#include <type_traits>
#include <os/utils/eventlog.hpp>

namespace sys {
    namespace observer {
        template<typename Algorithm, typename Filter, typename MotionModel, typename Trigger, typename... Sensors>
        class Observer {
            public:
                typedef Observer<Algorithm, Filter, MotionModel, Trigger, Sensors...> Self;
                typedef StateMessage<typename Filter::States> StateMessage;

            private:
                os::Dispatcher<Self, Trigger> dispatcher;

                /* Sensors */
                template<typename Sensor>
                struct SingleSensorWrapper {
                    os::Dispatcher<Self, Sensor> d;
                    explicit SingleSensorWrapper(Self*& self) : d(&Self::measurementUpdate<Sensor>, self) {
                        LOG_EVENT(typeid(Self).name(), 10, "Observing sensor " << os::demangle(typeid(Sensor).name()));
                    }
                };
                template<typename... WSensors>
                struct SensorWrapper : public SingleSensorWrapper<WSensors>... {
                    explicit SensorWrapper(Self* self) : SingleSensorWrapper<WSensors>(self)... {}
                };

                typename std::conditional<(sizeof...(Sensors) == 0),
                    Self*, SensorWrapper<Sensors...>>::type sensors;

            public:
                Filter filter;

                Observer();

                void timeUpdate(const Trigger);
                //~ FIXME: enable_if: void timeUpdate(const Trigger, typename MotionModel::Controls);

                template<typename Measurement, typename MUFilter = Algorithm>
                void measurementUpdate(const Measurement m) {
                    auto l = filter.retrieve_lock();
                    //~ std::cout << "Observer measurement update: " << typeid(Measurement).name() << std::endl;
                    MUFilter::template measurementUpdate(filter, m);
                    LOG_EVENT(typeid(Self).name(), 50, "MU: " << filter.state.transpose());
                }
        };
    }
}

#endif
