#pragma once
#ifndef SYS_MATH_FILTERING_EKF_HPP_
#define SYS_MATH_FILTERING_EKF_HPP_

#include <sys/math/filtering/GaussianFilter.hpp>
#include <Eigen/Cholesky>

namespace sys {
    namespace math {
        struct EKF {
            template<typename MotionModel, typename Filter>
            static void timeUpdate(Filter& filter, const typename Filter::Scalar dT) {
                auto A = MotionModel::systemJacobian(filter.state);

                filter.state = MotionModel::predict(filter.state, dT);
                filter.covariance = A*filter.covariance*A.transpose() + MotionModel::covariance(dT);
            }

            template<typename MotionModel, typename Filter>
            static void timeUpdate(Filter& filter, const typename MotionModel::Controls& u, const typename MotionModel::Scalar dT) {
                auto A = MotionModel::systemJacobian(filter.state);

                filter.state = MotionModel::predict(filter.state, u, dT);
                filter.covariance = A*filter.covariance*A.transpose() + MotionModel::covariance(dT);
            }

            template<typename Filter, typename Measurement>
            static void measurementUpdate(Filter& filter, const Measurement& measurement) {
                auto h = Measurement::Sensor::measurement(filter.state);
                auto H = Measurement::Sensor::jacobian(filter.state);

                auto KA = filter.covariance * H.transpose();

                auto KI = (H * filter.covariance * H.transpose() + measurement.R).ldlt();

                filter.state += KA * KI.solve(measurement.z - h);
                filter.covariance -= KA * KI.solve(H) * filter.covariance;
            }
        };
    }
}

#endif
