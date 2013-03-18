#pragma once
#ifndef SYS_MATH_MODELS_MOTION_HPP_
#define SYS_MATH_MODELS_MOTION_HPP_

#include <sys/math/models/motion/ConstantVelocities3D.hpp>
#include <sys/math/models/motion/DirectVelocities3D.hpp>


namespace sys {
    namespace math {
        namespace models {
            template<typename SDescription, typename CDescription>
            struct Description {
                typedef SDescription StateDescription;
                typedef CDescription ControlDescription;
                static const int nofStates      = StateDescription::nofStates;
                static const int nofControls    = ControlDescription::nofControls;
            };
        }
    }
}

#endif
