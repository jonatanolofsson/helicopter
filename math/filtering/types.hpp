#ifndef SYS_MATH_FILTERING_TYPES_HPP_
#define SYS_MATH_FILTERING_TYPES_HPP_

#include <sys/types.hpp>
#include <Eigen/Core>

namespace sys {
    namespace math {
        template<typename S, int N> struct Covariance       { typedef Eigen::Matrix<S, N, N> type; };
        template<typename S, int N> struct StateVector      { typedef Eigen::Matrix<S, N, 1> type; };
        template<typename S, int N> struct ControlVector    { typedef Eigen::Matrix<S, N, 1> type; };
    }
}

#endif
