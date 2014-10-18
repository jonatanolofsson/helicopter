#pragma once
#ifndef SYS_MATH_CONSTANTS_HPP_
#define SYS_MATH_CONSTANTS_HPP_

#define QUATERNION_ROTATION_FROM_ROTVEL(X,Y,Z) \
      0 , -(Z),  (Y), (X), \
     (Z),   0 ,  (X), (Y), \
    -(Y),  (X),   0 , (Z), \
    -(X), -(Y), -(Z),  0 ;

namespace sys {
    namespace math {
        namespace constants {
            static const double EPSILON = 1e-16;
            static const Scalar g = 9.82331;
        }
    }
}
#endif
