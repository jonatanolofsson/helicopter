#pragma once
#ifndef SYS_OBSERVER_HPP_
#define SYS_OBSERVER_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/Observer.hpp>
#include <type_traits>

namespace sys {
    namespace observer {
        template<typename Algorithm, typename Filter, typename MotionModel, typename Trigger, typename... Sensors>
        class Observer {
            public:
                typedef Observer<Algorithm, Filter, MotionModel, Trigger, Sensors...> Self;

            private:
                typename std::conditional<(MotionModel::ModelDescription::nofControls > 0),
                    os::Dispatcher<Self, Trigger, typename MotionModel::ModelDescription::ControlMessage>,
                    os::Dispatcher<Self, Trigger>>::type dispatcher;

                /* Sensors */
                template<typename Sensor>
                struct SingleSensorWrapper { 
                    os::Dispatcher<Self, Sensor> d;
                    explicit SingleSensorWrapper(Self*& self) : d(&Self::measurementUpdate<Sensor>, self) {}
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
                //~ void timeUpdate(const Trigger, typename MotionModel::Controls);

                template<typename Measurement, typename MUFilter = Algorithm>
                void measurementUpdate(const Measurement m) {
                    auto l = filter.retrieve_lock();
                    //~ std::cout << "Observer measurement update: " << typeid(Measurement).name() << std::endl;
                    MUFilter::template measurementUpdate(filter, m);
                }
        };
    }
}

#endif
