#pragma once

#include <sys/math/models.hpp>
#include <sys/math/states.hpp>
#include <sys/math/filtering.hpp>
#include <os/clock.hpp>

namespace sys {
    namespace observer {
        typedef math::models::HelicopterStates States;
        typedef math::models::HelicopterMotion<States> MotionModel;

        typedef math::EKF Algorithm;
        typedef math::GaussianFilter<States> Filter;
        typedef os::SystemTime TriggerType;

        typedef States::StateVector StateVector;

        namespace sensors {
            typedef math::GaussianMeasurement<math::models::Gps> Gps;
            typedef math::GaussianMeasurement<math::models::Imu<math::models::HelicopterImu>> Imu;
        }
    }
}
#include <sys/observer/Observer.hpp>

namespace sys {
#define OBSERVER_CLASS observer::Observer<observer::Algorithm, observer::Filter, observer::MotionModel, observer::TriggerType, observer::sensors::Gps>
    typedef OBSERVER_CLASS Observer;
}

