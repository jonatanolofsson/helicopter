#pragma once
#ifndef SYS_MATH_FILTERING_PARTICLE_HPP_
#define SYS_MATH_FILTERING_PARTICLE_HPP_

#include <Eigen/Core>
#include <sys/types.hpp>
#include <sys/math/statistics.hpp>

namespace sys {
    namespace math {
        using namespace Eigen;
        template<typename ModelDescription>
        struct Particle {
                typedef typename ModelDescription::States States;

                States state;
                typename ModelDescription::Scalar weight;
        };
    }
}

#endif
