#pragma once
#ifndef APP_SYS_MOTIONCONTROL_HPP_
#define APP_SYS_MOTIONCONTROL_HPP_

#include <sys/motioncontrol/MotionControl.hpp>
#include <sys/math/models.hpp>
#include <sys/math/states.hpp>
#include <sys/math/control/LqController.hpp>
#include <sys/com/statemessage.hpp>
#include <sys/Observer.hpp>

namespace sys {
    namespace motioncontrol {
        typedef math::models::XQ_3D States;
        typedef math::models::VW_3D Controls;

        typedef math::LqController<States, Controls, true> Controller;
        typedef math::models::Velocity_XQ_3D<math::models::XQ_3D> MotionModel;
        typedef Observer::StateMessage SystemStateMessage;
        typedef ReferenceMessage<States> Reference;
        typedef ControlMessage<Controls> ControlMessage;
    }

#define MOTIONCONTROL_CLASS motioncontrol::MotionControl<motioncontrol::Controller, motioncontrol::MotionModel, motioncontrol::SystemStateMessage, motioncontrol::Reference, motioncontrol::ControlMessage>
    typedef MOTIONCONTROL_CLASS MotionControl;
}
#endif
