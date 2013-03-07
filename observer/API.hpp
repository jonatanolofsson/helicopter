#ifndef SYS_OBSERVER_API_HPP_
#define SYS_OBSERVER_API_HPP_

#include <sys/observer/Observer.hpp>
#include <sys/math/models/models.hpp>
#include <sys/math/filtering/GaussianFilter.hpp>
#include <sys/math/filtering/EKF.hpp>
#include <os/clock.hpp>

namespace sys {
    namespace observer {
        typedef math::models::SCart3DAccQuat States;
        typedef math::models::CVel3 Controls;
        typedef math::models::Description<States, Controls> ModelDescription;
        typedef math::GaussianFilter<ModelDescription> SystemFilterState;
        typedef SystemFilterState::States SystemState;
        typedef models::motion::ConstantVelocities3D MotionModel;
        typedef Observer<math::EKF, MotionModel, SystemFilterState, os::SystemTime> ObserverType;
    }
}

#endif
