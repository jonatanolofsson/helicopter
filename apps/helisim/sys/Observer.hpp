#pragma once
#ifndef SYS_OBSERVER_API_HPP_
#define SYS_OBSERVER_API_HPP_

#include <sys/math/models.hpp>
#include <sys/math/filtering.hpp>
#include <os/clock.hpp>

namespace sys {
    namespace observer {
        typedef math::models::VWXQ_3D States;
        typedef math::models::Velocity_XQ_3D<States> MotionModel;

        typedef math::EKF Algorithm;
        typedef math::GaussianFilter<States> Filter;
        typedef os::SystemTime TriggerType;

        typedef States::StateVector StateVector;

        namespace sensors {
            typedef math::GaussianMeasurement<math::models::Gps> Gps;
        }
    }
}
#include <sys/observer/Observer.hpp>

namespace sys {
#define OBSERVER_CLASS observer::Observer<observer::Algorithm, observer::Filter, observer::MotionModel, observer::TriggerType, observer::sensors::Gps>
    typedef OBSERVER_CLASS Observer;
}

#endif
