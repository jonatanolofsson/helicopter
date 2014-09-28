#pragma once
#ifndef SYS_MATH_MODELS_MOTION_HPP_
#define SYS_MATH_MODELS_MOTION_HPP_

#include <sys/types.hpp>
#include <sys/math/filtering.hpp>
#include <sys/com/EigenMessage.hpp>

namespace sys {
    namespace math {
        namespace models {
            namespace messages {
                template<typename States> struct StateMessage : public EigenMessage<States> { StateMessage(){} explicit StateMessage(const States& s) : EigenMessage<States>(s) {} };
            }
        }
    }
}

#include <sys/math/models/motion/ConstantVelocities6D.hpp>

#endif
