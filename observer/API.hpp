#pragma once
#ifndef SYS_OBSERVER_API_HPP_
#define SYS_OBSERVER_API_HPP_

#include <sys/math/states.hpp>
#include <sys/math/models.hpp>
#include <sys/math/filtering.hpp>
#include <os/clock.hpp>

namespace sys {
    namespace observer {
        typedef math::models::SCart3DAccQuat                States;
        typedef math::models::CVel3                         Controls;
        typedef math::models::Description<States, Controls> StateDescription;

        typedef math::EKF                                   FilterType;
        typedef models::motion::ConstantVelocities3D        MotionModel;
        typedef math::GaussianFilter<StateDescription>      FilterState;
        typedef os::SystemTime                              TriggerType;

        typedef FilterState::States                         SystemState;

        namespace sensors {
            //~ typedef math::GaussianMeasurement<models::sensors::Gps>        Gps;
            typedef math::GaussianMeasurement<models::sensors::Imu>        Imu;
        }
    }
}

#include <sys/observer/Observer.hpp>

namespace sys {
    typedef observer::Observer<observer::FilterType, observer::MotionModel, observer::FilterState, observer::TriggerType> Observer;
}

#endif
