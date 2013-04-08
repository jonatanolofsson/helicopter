#pragma once
#ifndef SYS_MATH_MODELS_MOTION_HPP_
#define SYS_MATH_MODELS_MOTION_HPP_

#include <sys/types.hpp>
#include <sys/math/filtering.hpp>

namespace sys {
    namespace math {
        namespace models {
            template<typename SDescription, typename CDescription, typename S = Scalar>
            struct Description {
                typedef S Scalar;
                typedef SDescription StateDescription;
                typedef CDescription ControlDescription;

                static const int nofStates      = StateDescription::nofStates;
                static const int nofControls    = ControlDescription::nofControls;

                typedef typename StateVector<Scalar, nofStates>::Type States;
                typedef typename StateVector<Scalar, nofControls>::Type Controls;
            };
        }
    }
}

#include <sys/math/models/motion/ConstantVelocities3D.hpp>
#include <sys/math/models/motion/DirectVelocities3D.hpp>
#include <sys/math/models/motion/CoordinatedTurn2D.hpp>
#include <sys/math/models/motion/CoordinatedTurn2DPose.hpp>

#endif
