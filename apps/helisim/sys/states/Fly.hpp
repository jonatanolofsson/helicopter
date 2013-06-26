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
        : sc::simple_state<Fly, Top>
        {
            MotionControl motionController;
            ReferenceGenerator referenceGenerator;
            Simulator simulator;

            Fly();
        };
    }
}

#endif
