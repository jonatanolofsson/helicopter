#ifndef SYS_MATH_CONTROL_HPP_
#define SYS_MATH_CONTROL_HPP_

#include <Eigen/Core>

namespace sys {
    namespace math {
        template<typename S, int N> struct ControlVector    { typedef Eigen::Matrix<S, N, 1> type; };
    }
}


#include <sys/math/control/ControlState.hpp>
#include <sys/math/control/LqController.hpp>

#endif
