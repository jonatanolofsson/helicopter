#pragma once
#ifndef SYS_MOTIONCONTROL_API_HPP_
#define SYS_MOTIONCONTROL_API_HPP_

#include <sys/motioncontrol/MotionControl.hpp>
#include <sys/math/models.hpp>
#include <sys/math/states.hpp>
#include <sys/math/control/LqController.hpp>
#include <sys/Observer.hpp>

namespace sys {
    namespace motioncontrol {
        typedef math::models::SHelicopterControl        StateDescription;
        typedef math::models::CHelicopter               ControlDescription;
        typedef math::models::Description<StateDescription, ControlDescription> ModelDescription;

        typedef math::LqController<ModelDescription, 1>                Controller;
        typedef math::ControlState<ModelDescription>                ControlState;
        typedef math::models::HelicopterControl<ModelDescription>  ControlModel;
        typedef observer::SystemState                               SystemState;
        typedef math::models::Description<observer::ModelDescription::StateDescription, ControlDescription> SystemModelDescription;
    }

    typedef motioncontrol::MotionControl<motioncontrol::ModelDescription, motioncontrol::Controller, motioncontrol::ControlState, motioncontrol::ControlModel, motioncontrol::SystemState, motioncontrol::SystemModelDescription> MotionControl;
}
#endif
