#ifndef SYS_OBSERVER_IMPLEMENTATION_HPP_
#define SYS_OBSERVER_IMPLEMENTATION_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/observer/API.hpp>
#include <sys/settings.hpp>

namespace sys {
    namespace observer {
        template<typename Filter, typename Model, typename FilterState, typename Trigger>
        Observer<Filter, Model, FilterState, Trigger>::Observer()
        : dispatcher(&Self::timeUpdate, this)
        //~ , gps(&Self::measurementUpdate<sensors::Gps>, this)
        , imu(&Self::measurementUpdate<sensors::Imu>, this)
        {
            FilterState::Model::StateDescription::initialize(state);
        }

        template<typename Filter, typename Model, typename FilterState, typename Trigger>
        void Observer<Filter, Model, FilterState, Trigger>::timeUpdate(const Trigger) {
            auto l = state.retrieve_lock();
            Filter::template timeUpdate<Model>(state, settings::dT);
            os::yield(state.state);
        }
    }
}

#endif
