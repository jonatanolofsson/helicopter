#pragma once
#ifndef SYS_STATES_FLY_HPP_
#define SYS_STATES_FLY_HPP_
#include <sys/states/Top.hpp>
#include <sys/MotionControl.hpp>
#include <sys/ReferenceGenerator.hpp>
#include <sys/Simulator.hpp>

namespace sys {
    namespace states {
        struct Fly
        : sc::state<Fly, Top>
        {
            typedef sc::state<Fly, Top> Base;
            MotionControl motionController;
            ReferenceGenerator referenceGenerator;
            Simulator simulator;

            Fly(my_context);
        };
    }
}

#endif
