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
        typedef math::models::S2DPose     StateDescription;
        typedef math::models::CVW       ControlDescription;
        typedef math::models::Description<StateDescription, ControlDescription> ModelDescription;

        typedef math::LqController<ModelDescription>                Controller;
        typedef math::ControlState<ModelDescription>                ControlState;
        typedef math::models::CoordinatedTurn2DPose<ModelDescription>   ControlModel;
        typedef observer::SystemState                               SystemState;

        typedef ModelDescription::States                            Reference;
        typedef ModelDescription::Controls                          Controls;
    }

    typedef motioncontrol::MotionControl<motioncontrol::Controller, motioncontrol::ControlState, motioncontrol::ControlModel, motioncontrol::SystemState> MotionControl;
}
#endif
