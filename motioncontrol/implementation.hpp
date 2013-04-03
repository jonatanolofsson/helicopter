#pragma once
#ifndef SYS_MOTIONCONTROL_IMPLEMENTATION_HPP_
#define SYS_MOTIONCONTROL_IMPLEMENTATION_HPP_

#include <sys/settings.hpp>
#include <os/com/Dispatcher.hpp>
#include <sys/types.hpp>
#include <sys/math/control/LqController.hpp>

#include <sys/math/filtering/GaussianFilter.hpp>
#include <sys/math/states.hpp>
#include <sys/math/models.hpp>

#include <sys/observer/API.hpp>
#include <sys/motioncontrol/API.hpp>

namespace sys {
    namespace motioncontrol {
        template<typename T> struct ZeroVector { static T z; ZeroVector() { z.setZero(); } };
        template<typename T> T ZeroVector<T>::z;

        template<typename ControllerType, typename ControlState, typename ControlModel, typename SystemState>
        void MotionControl<ControllerType, ControlState, ControlModel, SystemState>
        ::updateControl(const SystemState systemState) {
            typedef typename ControlModel::Controls Controls;
            ControlState controlState(systemState);
            controller.template updateModel<ControlModel::isDiscrete>(ControlModel::systemJacobian(controlState.state, ZeroVector<Controls>::z), ControlModel::controlJacobian(controlState.state, ZeroVector<Controls>::z));
            os::yield(controller(controlState.state));
        }

        template<typename ControllerType, typename ControlState, typename ControlModel, typename SystemState>
        MotionControl<ControllerType, ControlState, ControlModel, SystemState>
        ::MotionControl()
        : dispatcher(&Self::updateControl, this)
        {}
    }
}

#endif
