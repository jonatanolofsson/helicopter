#pragma once
#ifndef SYS_SIMULATOR_IMPLEMENTATION_HPP_
#define SYS_SIMULATOR_IMPLEMENTATION_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/simulator/API.hpp>
#include <sys/settings.hpp>

namespace sys {
    namespace simulator {
        template<typename MotionModel_, typename Filter_>
        Simulator<MotionModel_, Filter_>::Simulator()
        : dispatcher(&Self::simulate, this)
        {
            MotionModel::ModelDescription::StateDescription::initialize(filter);
        }

        template<typename MotionModel_, typename Filter_>
        void Simulator<MotionModel_, Filter_>::simulate(const Controls u) {
            filter.state = MotionModel::predict(filter.state, u, settings::dT) + filter.noise();
        }
    }
}

#endif
