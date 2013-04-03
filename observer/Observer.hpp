#pragma once
#ifndef SYS_OBSERVER_HPP_
#define SYS_OBSERVER_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/observer/API.hpp>
#include <type_traits>

namespace sys {
    namespace observer {
        template<typename Algorithm, typename Filter, typename MotionModel, typename Trigger>
        class Observer {
            public:
                typedef Observer<Algorithm, Filter, MotionModel, Trigger> Self;
            private:
                typename std::conditional<(MotionModel::ModelDescription::nofControls > 0),
                    os::Dispatcher<Self, Trigger, typename MotionModel::Controls>,
                    os::Dispatcher<Self, Trigger>>::type dispatcher;

                /* Sensors */
                //~ os::Dispatcher<Observer, sensors::Gps> gps;
                //~ os::Dispatcher<Self, sensors::Imu> imu;

            public:
                Filter filter;

                Observer();

                void timeUpdate(const Trigger);
                //~ void timeUpdate(const Trigger, typename MotionModel::Controls);

                template<typename Measurement, typename MUFilter = Filter>
                void measurementUpdate(const Measurement m) {
                    auto l = filter.retrieve_lock();
                    //~ std::cout << "Observer measurement update: " << typeid(Measurement).name() << std::endl;
                    MUFilter::template measurementUpdate(filter, m);
                }
        };
    }
}

#endif
