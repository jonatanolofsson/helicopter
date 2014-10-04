#pragma once
#ifndef SYS_OBSERVER_API_HPP_
#define SYS_OBSERVER_API_HPP_

#include <sys/math/models.hpp>
#include <sys/math/filtering.hpp>
#include <os/clock.hpp>

namespace sys {
    namespace observer {
        typedef math::models::SCart3DQuat<>                 StateDescription;
        typedef math::models::ConstantVelocities6D<StateDescription> MotionModel;

        typedef math::EKF                                   Algorithm;
        typedef math::GaussianFilter<StateDescription>      Filter;
        typedef os::SystemTime                              TriggerType;

        typedef StateDescription::StateVector               SystemState;

        namespace sensors {
            typedef math::GaussianMeasurement<math::models::Gps<StateDescription>> Gps;
        }
    }
}
#include <sys/observer/Observer.hpp>

namespace sys {
#define OBSERVER_CLASS observer::Observer<observer::Algorithm, observer::Filter, observer::MotionModel, observer::TriggerType, observer::sensors::Gps> 
    typedef OBSERVER_CLASS Observer;
}

#endif
