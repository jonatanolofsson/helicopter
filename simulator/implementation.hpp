#ifndef SYS_SIMULATOR_IMPLEMENTATION_HPP_
#define SYS_SIMULATOR_IMPLEMENTATION_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/simulator/API.hpp>
#include <sys/settings.hpp>

namespace sys {
    namespace simulator {
        template<typename MotionModel_, typename FilterState_>
        Simulator<MotionModel_, FilterState_>::Simulator()
        : dispatcher(&Self::simulate, this)
        {
            FilterState::Model::StateDescription::initialize(state);
        }

        template<typename MotionModel_, typename FilterState_>
        void Simulator<MotionModel_, FilterState_>::simulate(const ControlState u) {
            state.controls = u;
            state.state = MotionModel::predict(state, settings::dT) + state.noise();
        }
    }
}

#endif
