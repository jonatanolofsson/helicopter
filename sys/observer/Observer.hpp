#pragma once

#include <os/com/Dispatcher.hpp>
#include <sys/Observer.hpp>
#include <type_traits>
#include <os/utils/eventlog.hpp>

namespace sys {
    namespace observer {
        template<typename Algorithm_, typename Filter_, typename MotionModel_, typename Trigger, typename... Sensors>
        class Observer {
            public:
                typedef Algorithm_ Algorithm;
                typedef Filter_ Filter;
                typedef MotionModel_ MotionModel;
                typedef Observer<Algorithm, Filter, MotionModel, Trigger, Sensors...> Self;
                typedef StateMessage<typename Filter::States> StateMessage;

            private:
                Filter& filter;
                os::Dispatcher<Self, Trigger> dispatcher;

                /* Sensors */
                template<typename Sensor>
                struct SingleSensorWrapper {
                    os::AsynchronousDispatcher<Self, Sensor> d;
                    explicit SingleSensorWrapper(Self*& self) : d(&Self::template measurementUpdate<Sensor>, self) {
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
                explicit Observer(Filter&);

                void timeUpdate(const Trigger);

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

