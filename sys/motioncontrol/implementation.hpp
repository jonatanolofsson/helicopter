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

        template<typename ModelDescription, typename ControllerType, typename ControlState, typename ControlModel, typename SystemState, typename SystemModelDescription>
        void MotionControl<ModelDescription, ControllerType, ControlState, ControlModel, SystemState, SystemModelDescription>
        ::updateControl(const SystemState systemState, const ReferenceMessage referenceMessage) {
            ControlState controlState(systemState);
            auto A = ControlModel::template systemJacobian<SystemModelDescription>(systemState, control);
            ExtendedStateMatrix F; F.setZero();
            F.template block<ModelDescription::nofStates, ModelDescription::nofStates>(0,0) = A;
            F.template block<ModelDescription::nofStates, 1>(0, ModelDescription::nofStates) = ControlModel::template derivative<SystemModelDescription>(systemState, control) - A*controlState.state;
            F(ModelDescription::nofStates, ModelDescription::nofStates) = 1e-9;

            ExtendedControlMatrix B; B.setZero();
            B.template block<ModelDescription::nofStates, ModelDescription::nofControls>(0,0) = ControlModel::template controlJacobian<SystemModelDescription>(systemState, control);
            controller.template updateModel<ControlModel::isDiscrete>(F, B);
            LOG_EVENT(typeid(Self).name(), 50, "Controlstate: " << controlState.state.transpose());
            LOG_EVENT(typeid(Self).name(), 50, "Reference: " << referenceMessage.value.transpose());

            ExtendedControlState extendedControlState; extendedControlState << controlState.state - referenceMessage.value, 1;
            /*
             *static_assert(12 == ControllerType::StateVector::RowsAtCompileTime, "Wrong size of controlmodel!");
             *static_assert(12 == ExtendedControlState::RowsAtCompileTime, "Wrong size of controlstate!");
             *static_assert(1 == ControllerType::StateVector::ColsAtCompileTime, "Wrong size of controlmodel!");
             *static_assert(1 == ExtendedControlState::ColsAtCompileTime, "Wrong size of controlstate!");
             */
            os::yield(typename ControlModel::ModelDescription::ControlMessage(controller(extendedControlState)));
        }

        template<typename ModelDescription, typename ControllerType, typename ControlState, typename ControlModel, typename SystemState, typename SystemModelDescription>
        MotionControl<ModelDescription, ControllerType, ControlState, ControlModel, SystemState, SystemModelDescription>
        ::MotionControl()
        : dispatcher(&Self::updateControl, this)
        {}
    }
}

#endif
