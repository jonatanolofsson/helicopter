#pragma once
#ifndef SYS_MATH_FILTERING_PARTICLE_FILTER_HPP_
#define SYS_MATH_FILTERING_PARTICLE_FILTER_HPP_

#include <os/mem/ProtectedData.hpp>
#include <sys/math/statistics.hpp>
#include <sys/math/filtering.hpp>
#include <sys/math/control.hpp>
#include <sys/types.hpp>

namespace sys {
    namespace math {
        template<typename ModelDescription_, unsigned N>
        struct ParticleFilter : public os::ProtectedClass {
            static const unsigned nofParticles = N;
            typedef ModelDescription_ ModelDescription;
            typedef typename ModelDescription::States States;
            typedef typename ModelDescription::Scalar Scalar;
            typedef Particle<ModelDescription> ParticleType;

            States state;
            ParticleType particles[N];
            Scalar maxWeight;

            explicit ParticleFilter() {
                state.setZero();
            }
        };
    }
}

#endif
