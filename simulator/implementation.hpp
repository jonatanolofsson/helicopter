#pragma once
#ifndef SYS_SIMULATOR_IMPLEMENTATION_HPP_
#define SYS_SIMULATOR_IMPLEMENTATION_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/Simulator.hpp>
#include <sys/settings.hpp>
#include <os/utils/eventlog.hpp>

#include <iostream>

namespace sys {
    namespace simulator {
        template<typename MotionModel, typename... Sensors>
        Simulator<MotionModel, Sensors...>::Simulator()
        : d(&Self::simulate, this)
        {
            MotionModel::ModelDescription::StateDescription::initializeState(state);
        }

        template<typename MotionModel, typename... Sensors>
        void Simulator<MotionModel, Sensors...>::simulate(const typename MotionModel::ModelDescription::ControlMessage u) {
            state = MotionModel::predict(state, u.value, settings::dT);
            LOG_EVENT(typeid(Self).name(), 50, "Simulated state: " << state.transpose());
            yieldSensorReadings<Sensors...>();
        }
    }
}

#endif
