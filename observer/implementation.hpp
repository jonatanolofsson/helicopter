#pragma once
#ifndef SYS_OBSERVER_IMPLEMENTATION_HPP_
#define SYS_OBSERVER_IMPLEMENTATION_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/Observer.hpp>
#include <sys/settings.hpp>

#include <iostream>

namespace sys {
    namespace observer {
        template<typename Algorithm, typename Filter, typename MotionModel, typename Trigger>
        Observer<Algorithm, Filter, MotionModel, Trigger>::Observer()
        : dispatcher(&Self::timeUpdate, this)
        //~ , gps(&Self::measurementUpdate<sensors::Gps>, this)
        , imu(&Self::measurementUpdate<sensors::Imu>, this)
        , mouse(&Self::measurementUpdate<sensors::Mouse>, this)
        , pfilter(&Self::measurementUpdate<sensors::ParticleFilterSensor>, this)
        {
            MotionModel::ModelDescription::StateDescription::initialize(filter);
        }

        template<typename Algorithm, typename Filter, typename MotionModel, typename Trigger>
        void Observer<Algorithm, Filter, MotionModel, Trigger>::timeUpdate(const Trigger) {
            auto l = filter.retrieve_lock();
            Algorithm::template timeUpdate<MotionModel>(filter, settings::dT);
            os::yield(filter.state);
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
