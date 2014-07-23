#pragma once
#ifndef SYS_MATH_FILTERING_HPP_
#define SYS_MATH_FILTERING_HPP_

#include <Eigen/Core>

namespace sys {
    namespace math {
        template<typename S, int N> using StateVector = Eigen::Matrix<S, N, 1>;
        template<typename S, int N> using Covariance = Eigen::Matrix<S, N, N>;
    }
}

#include <sys/math/filtering/EKF.hpp>
#include <sys/math/filtering/PF.hpp>

#endif
