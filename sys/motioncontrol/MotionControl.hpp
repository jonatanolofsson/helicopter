#pragma once
#ifndef SYS_MOTIONCONTROL_HPP_
#define SYS_MOTIONCONTROL_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/Observer.hpp>

namespace sys {
    namespace motioncontrol {
        template<typename ModelDescription, typename ControllerType, typename ControlState, typename ControlModel, typename SystemState, typename SystemModelDescription>
        class MotionControl {
            public:
                typedef MotionControl<ModelDescription, ControllerType, ControlState, ControlModel, SystemState, SystemModelDescription> Self;
                typedef typename ModelDescription::ReferenceMessage ReferenceMessage;
                typedef typename ControlModel::Controls Controls;
                typedef Eigen::Matrix<typename ModelDescription::Scalar, ModelDescription::nofStates+1, ModelDescription::nofStates+1> ExtendedStateMatrix;
                typedef Eigen::Matrix<typename ModelDescription::Scalar, ModelDescription::nofStates+1, ModelDescription::nofControls> ExtendedControlMatrix;
                typedef Eigen::Matrix<typename ModelDescription::Scalar, ModelDescription::nofStates+1, 1> ExtendedControlState;

            private:
                ControllerType controller;
                os::Dispatcher<Self, SystemState, ReferenceMessage> dispatcher;
                Controls control;

                void updateControl(const SystemState systemState, const ReferenceMessage referenceMessage);

            public:
                MotionControl();
        };
    }
}

#endif
