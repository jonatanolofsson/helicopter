#ifndef SYS_OBSERVER_API_HPP_
#define SYS_OBSERVER_API_HPP_

#include <sys/math/models/models.hpp>
#include <sys/math/filtering/GaussianFilter.hpp>
#include <sys/math/filtering/EKF.hpp>
#include <os/clock.hpp>

#include <sys/observer/gps.hpp>

namespace sys {
    namespace observer {
        typedef math::models::SCart3DAccQuat                States;
        typedef math::models::CVel3                         Controls;
        typedef math::models::Description<States, Controls> ModelDescription;

        typedef math::EKF                                   FilterType;
        typedef models::motion::ConstantVelocities3D        MotionModel;
        typedef math::GaussianFilter<ModelDescription>      FilterState;
        typedef os::SystemTime                              TriggerType;

        typedef FilterState::States                         SystemState;

        namespace sensors {
            typedef math::GaussianMeasurement<GPS<>>        GPS;
        }
    }
}

#include <sys/observer/Observer.hpp>

namespace sys {
    typedef observer::Observer<observer::FilterType, observer::MotionModel, observer::FilterState, observer::TriggerType> Observer;
}

#endif
