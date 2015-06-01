#pragma once
#ifndef SYS_OBSERVER_IMPLEMENTATION_HPP_
#define SYS_OBSERVER_IMPLEMENTATION_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/Observer.hpp>
#include <sys/settings.hpp>

#include <iostream>

namespace sys {
    namespace observer {
        template<typename Algorithm, typename Filter, typename MotionModel, typename Trigger, typename... Sensors>
        Observer<Algorithm, Filter, MotionModel, Trigger, Sensors...>::Observer(Filter& filter_)
        : filter(filter_)
        , dispatcher(&Self::timeUpdate, this)
        , sensors(this)
        {
            MotionModel::States::initialize(filter);
            //LOG_EVENT(typeid(Self).name(), 10, "Initialized observer with " << sizeof...(Sensors) << " sensors.");
        }

        template<typename Algorithm, typename Filter, typename MotionModel, typename Trigger, typename... Sensors>
        void Observer<Algorithm, Filter, MotionModel, Trigger, Sensors...>::timeUpdate(const Trigger) {
            auto l = filter.retrieve_lock();
            Algorithm::template timeUpdate<MotionModel>(filter, settings::dT);
            //LOG_EVENT(typeid(Self).name(), 10, "Updated observer: " << filter.state.transpose());
            os::yield(StateMessage(filter.state));
        }

        //~ template<typename Algorithm, typename Filter, typename MotionModel, typename Trigger>
        //~ void Observer<Algorithm, Filter, MotionModel, Trigger>::timeUpdate(const Trigger, const typename MotionModel::Controls u) {
            //~ auto l = filter.retrieve_lock();
            //~ Algorithm::template timeUpdate<MotionModel>(filter, u, settings::dT);
            //~ os::yield(filter.state);
        //~ }
    }
}

#endif
