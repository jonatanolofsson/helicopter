#pragma once
#ifndef SYS_MATH_CONSTANTS_HPP_
#define SYS_MATH_CONSTANTS_HPP_

#define USING_XYZ static const int X = 0; static const int Y = 1; static const int Z = 2;
#define USING_XYZW USING_XYZ static const int W = 3;

#define QUATERNION_ROTATION_FROM_ROTVEL(X,Y,Z) \
      0 , -(Z),  (Y), (X), \
     (Z),   0 ,  (X), (Y), \
    -(Y),  (X),   0 , (Z), \
    -(X), -(Y), -(Z),  0 ;

namespace sys {
    namespace math {
        static const double EPSILON = 1e-16;
    }
}
#endif
