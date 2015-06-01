#pragma once

#include <os/com/Dispatcher.hpp>
#include <sys/Simulator.hpp>
#include <sys/settings.hpp>
#include <os/utils/eventlog.hpp>
#include <sys/com/statemessage.hpp>

#include <iostream>

namespace sys {
    namespace simulator {
        template<typename GlobalFilter, typename MotionModel, typename ControlMessage, typename... Sensors>
        Simulator<GlobalFilter, MotionModel, ControlMessage, Sensors...>::Simulator(GlobalFilter& filter_)
        : filter(filter_)
        , d(&Self::simulate, this)
        {
            MotionModel::States::initializeState(state);
        }

        template<typename GlobalFilter, typename MotionModel, typename ControlMessage, typename... Sensors>
        void Simulator<GlobalFilter, MotionModel, ControlMessage, Sensors...>::simulate(const ControlMessage u) {
            ControlMessage::States::template update<typename MotionModel::States>(state, u.value);
            state = MotionModel::template predict<States>(state, settings::dT);
            //LOG_EVENT(typeid(Self).name(), 50, "Simulated state: " << state.transpose());
            yieldSensorReadings<Sensors...>();
            os::yield(SimulatedStateMessage(state));
        }
    }
}

