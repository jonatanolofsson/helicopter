#pragma once
#ifndef SYS_SIMULATOR_API_HPP_
#define SYS_SIMULATOR_API_HPP_

#include <sys/math/states.hpp>
#include <sys/math/models.hpp>
#include <sys/math/filtering.hpp>
#include <os/clock.hpp>
#include <sys/motioncontrol/API.hpp>
#include <sys/math/algorithm.hpp>

namespace sys {
    namespace simulator {
        typedef math::models::S2DPose     StateDescription;
        typedef motioncontrol::ControlDescription           ControlDescription;
        typedef math::models::Description<StateDescription, ControlDescription> ModelDescription;

        typedef math::models::CoordinatedTurn2DPose<ModelDescription> MotionModel;
        typedef math::GaussianFilter<ModelDescription>      Filter;

        static const Scalar distanceScaling = 1000.0;

        namespace sensors {
            typedef math::models::Gps<ModelDescription>     Gps;
            typedef math::models::Imu<ModelDescription>     Imu;
        }
    }
}

#include <sys/simulator/Simulator.hpp>

namespace sys {
    typedef simulator::Simulator<simulator::MotionModel, simulator::Filter> Simulator;
}

#endif
