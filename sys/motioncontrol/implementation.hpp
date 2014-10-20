#pragma once
#ifndef SYS_MOTIONCONTROL_IMPLEMENTATION_HPP_
#define SYS_MOTIONCONTROL_IMPLEMENTATION_HPP_

#include <Eigen/Core>

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
        using namespace Eigen;
        template<typename Algorithm, typename MotionModel, typename SystemStateMessage, typename ReferenceMessage, typename ControlMessage>
        void MotionControl<Algorithm, MotionModel, SystemStateMessage, ReferenceMessage, ControlMessage>
        ::updateControl(const SystemStateMessage systemState, const ReferenceMessage reference) {
            static const int nofStates = Algorithm::nofStates;
            static const int nofControls = Algorithm::nofControls;
            typedef Matrix<Scalar, nofStates, nofStates> ExtendedStateMatrix;
            typedef Matrix<Scalar, nofStates, nofControls> ExtendedControlMatrix;
            typedef Matrix<Scalar, nofStates, 1> ExtendedControlState;
            ExtendedStateMatrix F; F.setZero();
            ExtendedControlMatrix B; B.setZero();
            auto controlState = Algorithm::States::template translate<SystemStates>(systemState.value);
            static const int linState = Algorithm::linState;

            auto A = math::template differentiate<MotionModel, typename Algorithm::States, SystemStates>(systemState.value);
            F.template block<MotionModel::nofStates, MotionModel::nofStates>(0,0) = A;
            F.template block<MotionModel::nofStates, 1>(0, linState) =
                    MotionModel::template predict<SystemStates>(systemState.value)
                    - A*controlState;
            if(MotionModel::isDiscrete) {
                F(linState, linState) = 1-1e-5;
            } else {
                F(linState, linState) = -1e-9;
            }

            B.template block<MotionModel::nofStates, Algorithm::nofControls>(0,0) =
                    math::template differentiate<MotionModel, typename Algorithm::Controls, SystemStates>(systemState.value);
            controller.template updateModel<MotionModel::isDiscrete>(F, B);
            LOG_EVENT(typeid(Self).name(), 50, "Controlstate: " << controlState.transpose());
            LOG_EVENT(typeid(Self).name(), 50, "Reference: " << reference.value.transpose());

            ExtendedControlState extendedControlState; extendedControlState << controlState - reference.value, 1;
            /*
             *static_assert(12 == ControllerType::StateVector::RowsAtCompileTime, "Wrong size of controlmodel!");
             *static_assert(12 == ExtendedControlState::RowsAtCompileTime, "Wrong size of controlstate!");
             *static_assert(1 == ControllerType::StateVector::ColsAtCompileTime, "Wrong size of controlmodel!");
             *static_assert(1 == ExtendedControlState::ColsAtCompileTime, "Wrong size of controlstate!");
             */
            auto u = controller.direct_eval(extendedControlState);
            os::yield(ControlMessage(u.template segment<ControlMessage::States::nofStates>(0)));
        }

        template<typename Algorithm, typename MotionModel, typename SystemStates, typename Reference, typename ControlMessage>
        MotionControl<Algorithm, MotionModel, SystemStates, Reference, ControlMessage>
        ::MotionControl()
        : dispatcher(&Self::updateControl, this)
        {}
    }
}

#endif
