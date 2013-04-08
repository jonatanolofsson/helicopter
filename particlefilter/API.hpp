#pragma once
#ifndef SYS_PARTICLEFILTER_API_HPP_
#define SYS_PARTICLEFILTER_API_HPP_

#include <sys/settings.hpp>
#include <sys/motioncontrol/API.hpp>
#include <sys/math/models.hpp>
#include <os/clock.hpp>
#include <sys/com/StmMessages.hpp>

namespace sys {
    namespace particlefilter {
        static const int nofParticles = 5000;
        typedef math::models::S2DPose StateDescription;
        typedef math::models::CVW ControlDescription;
        typedef math::models::Description<StateDescription, ControlDescription> ModelDescription;
        typedef math::models::CoordinatedTurn2DPose<ModelDescription> MotionModel;
        typedef math::ParticleFilter<ModelDescription, nofParticles> Filter;
        typedef math::PF Algorithm;

        typedef stm::SensorMessage                          SensorMessage;
        typedef motioncontrol::ModelDescription::Controls   ControlMessage;
        typedef os::SystemTime                              TimeTrigger;

        static const Scalar dT = sys::settings::dT;

    }
}


#include <sys/particlefilter/ParticleFilter.hpp>

#endif
