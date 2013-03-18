#pragma once
#ifndef SYS_MATH_FILTERING_HPP_
#define SYS_MATH_FILTERING_HPP_

#include <Eigen/Core>

namespace sys {
    namespace math {
        template<typename S, int N> struct Covariance       { typedef Eigen::Matrix<S, N, N> type; };
        template<typename S, int N> struct StateVector      { typedef Eigen::Matrix<S, N, 1> type; };
    }
}

#include <sys/math/filtering/EKF.hpp>

#endif
