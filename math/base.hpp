#pragma once
#ifndef SYS_MATH_BASE_HPP_
#define SYS_MATH_BASE_HPP_

#include <cmath>

#define SQUARE(a) ((a)*(a))
#define countof(a) (sizeof(a)/sizeof(a[0]))

namespace sys {
    namespace math {
        template<typename S = Scalar>
        S toPi(S a) {
            return std::fmod(a + M_PI, 2*M_PI) - M_PI;
        }

        template<typename D1, typename D2, typename S = Scalar>
        S angleFromTo(const D1 from, const D2 to) {
            return std::atan2(to[1]-from[1], to[0]-from[0]);
        }

    }
}

#endif
