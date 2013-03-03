#ifndef SYS_MATH_MODELS_MODELS_HPP_
#define SYS_MATH_MODELS_MODELS_HPP_

#include "ConstantVelocities3D.hpp"
#include "DirectVelocities3D.hpp"

#include "SCart3DQuat.hpp"
#include "SCart3D.hpp"
#include "CVel3.hpp"
#include "C6VW.hpp"
#include "NoControl.hpp"

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
