#pragma once

#include <sys/motioncontrol/MotionControl.hpp>
#include <sys/math/models.hpp>
#include <sys/math/states.hpp>
#include <sys/math/control/LqController.hpp>
#include <sys/com/statemessage.hpp>
#include <sys/Observer.hpp>

namespace sys {
    namespace motioncontrol {
        typedef math::models::VWQi_3D States;
        typedef math::models::HelicopterControls Controls;

        typedef Observer::Filter GlobalFilter;
        typedef math::LqController<States, Controls, true> Controller;
        typedef math::models::Helicopter<States> MotionModel;
        typedef Observer::StateMessage SystemStateMessage;
        typedef ReferenceMessage<States> Reference;
        typedef ControlMessage<Controls> ControlMessage;
    }

#define MOTIONCONTROL_CLASS motioncontrol::MotionControl<motioncontrol::GlobalFilter, motioncontrol::Controller, motioncontrol::MotionModel, motioncontrol::SystemStateMessage, motioncontrol::Reference, motioncontrol::ControlMessage>
    typedef MOTIONCONTROL_CLASS MotionControl;
}

