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
        typedef math::models::SHelicopterControl        States;
        typedef math::models::CHelicopter               Controls;

        typedef math::LqController<States, Controls> Controller;
        typedef math::models::HelicopterControl MotionModel;
        typedef observer::SystemState SystemState;
    }

    typedef motioncontrol::MotionControl<motioncontrol::Controller, motioncontrol::MotionModel, motioncontrol::SystemState> MotionControl;
}
#endif
