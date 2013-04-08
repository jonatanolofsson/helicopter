#pragma once
#ifndef SYS_MATH_BASE_HPP_
#define SYS_MATH_BASE_HPP_

#define SQUARE(a) ((a)*(a))

namespace sys {
    namespace math {
        template<typename S = Scalar>
        S toPi(S a) {
            return std::fmod(a + M_PI, 2*M_PI) - M_PI;
        }
    }
}

#endif
