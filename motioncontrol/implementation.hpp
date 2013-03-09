#ifndef SYS_MOTIONCONTROL_IMPLEMENTATION_HPP_
#define SYS_MOTIONCONTROL_IMPLEMENTATION_HPP_

#include <sys/settings.hpp>
#include <os/com/Dispatcher.hpp>
#include <sys/types.hpp>
#include <sys/math/control/LqController.hpp>

#include <sys/math/filtering/GaussianFilter.hpp>
#include <sys/math/models/SCart3DAccQuat.hpp>
#include <sys/math/models/ConstantVelocities3D.hpp>

#include <sys/observer/API.hpp>
#include <sys/motioncontrol/API.hpp>

namespace sys {
    namespace motioncontrol {
        template<typename ControllerType, typename ControlState, typename Model, typename Trigger>
        void MotionControl<ControllerType, ControlState, Model, Trigger>
        ::updateControl(const Trigger systemState) {
            ControlState controlState(systemState);
            controller.template updateModel<Model::CD>(Model::systemJacobian(controlState), Model::controlJacobian(controlState));
            os::yield(controller(controlState.state));
        }

        template<typename ControllerType, typename ControlState, typename Model, typename Trigger>
        MotionControl<ControllerType, ControlState, Model, Trigger>
        ::MotionControl()
        : dispatcher(&Self::updateControl, this)
        {}
    }
}

#endif
