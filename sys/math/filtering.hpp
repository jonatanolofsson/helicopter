#pragma once
#ifndef SYS_MATH_FILTERING_HPP_
#define SYS_MATH_FILTERING_HPP_

#include <Eigen/Core>
#include <vector>
#include <sys/types.hpp>

namespace sys {
    namespace math {
        namespace internal {
            template<int N> using StateVector = Eigen::Matrix<Scalar, N, 1>;
            template<int N> using Covariance = Eigen::Matrix<Scalar, N, N>;
            typedef std::vector<int> StateMap;
        }
    }
}

#include <sys/math/filtering/EKF.hpp>
#include <sys/math/filtering/PF.hpp>

#endif
