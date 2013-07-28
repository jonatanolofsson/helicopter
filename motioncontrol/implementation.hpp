#pragma once
#ifndef SYS_MOTIONCONTROL_IMPLEMENTATION_HPP_
#define SYS_MOTIONCONTROL_IMPLEMENTATION_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/types.hpp>
#include <sys/math/control/LqController.hpp>

#include <sys/math/filtering/GaussianFilter.hpp>
#include <sys/math/states.hpp>
#include <sys/math/models.hpp>

#include <sys/Observer.hpp>
#include <sys/MotionControl.hpp>

namespace sys {
    namespace motioncontrol {
        template<typename T> struct ZeroVector { static T z; ZeroVector() { z.setZero(); } };
        template<typename T> T ZeroVector<T>::z;

        template<typename ModelDescription, typename ControllerType, typename ControlState, typename ControlModel, typename SystemState>
        void MotionControl<ModelDescription, ControllerType, ControlState, ControlModel, SystemState>
        ::updateControl(const SystemState systemState, const ReferenceMessage referenceMessage) {
            ControlState controlState(systemState);
            controller.template updateModel<ControlModel::isDiscrete>(ControlModel::systemJacobian(controlState.state, ZeroVector<typename ControlModel::Controls>::z), ControlModel::controlJacobian(controlState.state, ZeroVector<typename ControlModel::Controls>::z));
            LOG_EVENT(typeid(Self).name(), 50, "Controlstate: " << controlState.state.transpose());
            LOG_EVENT(typeid(Self).name(), 50, "Reference: " << referenceMessage.value.transpose());
            os::yield(typename ControlModel::ModelDescription::ControlMessage(controller(controlState.state, referenceMessage.value)));
        }

        template<typename ModelDescription, typename ControllerType, typename ControlState, typename ControlModel, typename SystemState>
        MotionControl<ModelDescription, ControllerType, ControlState, ControlModel, SystemState>
        ::MotionControl()
        : dispatcher(&Self::updateControl, this)
        {}
    }
}

#endif
