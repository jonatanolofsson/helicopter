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
        template<typename MotionModel, typename ControlMessage, typename... Sensors>
        Simulator<MotionModel, ControlMessage, Sensors...>::Simulator()
        : d(&Self::simulate, this)
        {
            MotionModel::States::initializeState(state);
        }

        template<typename MotionModel, typename ControlMessage, typename... Sensors>
        void Simulator<MotionModel, ControlMessage, Sensors...>::simulate(const ControlMessage u) {
            ControlMessage::States::template update<States>(state, u.value);

            state = MotionModel::template predict<States>(state, settings::dT);
            LOG_EVENT(typeid(Self).name(), 50, "Simulated state: " << state.transpose());
            yieldSensorReadings<Sensors...>();
        }
    }
}

#endif
