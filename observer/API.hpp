#pragma once
#ifndef SYS_OBSERVER_API_HPP_
#define SYS_OBSERVER_API_HPP_

#include <sys/math/states.hpp>
#include <sys/math/models.hpp>
#include <sys/math/filtering.hpp>
#include <os/clock.hpp>

namespace sys {
    namespace observer {
        typedef math::models::SCT2D                         StateDescription;
        typedef math::models::C0                            ControlDescription;
        typedef math::models::Description<StateDescription, ControlDescription> ModelDescription;

        typedef math::EKF                                   Algorithm;
        typedef math::GaussianFilter<ModelDescription>      Filter;
        typedef math::models::CoordinatedTurn2D<ModelDescription> MotionModel;
        typedef os::SystemTime                              TriggerType;

        typedef ModelDescription::States                    SystemState;

        namespace sensors {
            typedef math::GaussianMeasurement<math::models::Imu1d<ModelDescription>>        Imu;
        }
    }
}

#include <sys/observer/Observer.hpp>

namespace sys {
    typedef observer::Observer<observer::Algorithm, observer::Filter, observer::MotionModel, observer::TriggerType> Observer;
}

#endif
