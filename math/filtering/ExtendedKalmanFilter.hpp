#ifndef SYS_MATH_FILTERING_EXTENDED_KALMAN_FILTER_HPP_
#define SYS_MATH_FILTERING_EXTENDED_KALMAN_FILTER_HPP_

#include <sys/math/filtering/KalmanFilter.hpp>
#include <Eigen/Cholesky>
#include <iostream>

namespace sys {
    namespace math {
        struct ExtendedKalmanFilter {
            template<typename MotionModel, typename T>
            static void timeUpdate(T& filter) {
                auto l = filter.retrieve_lock();
                auto A = MotionModel::systemJacobian(filter);

                filter.state = MotionModel::predict(filter);
                filter.covariance = A*filter.covariance*A.transpose() + MotionModel::template Q<T>();
            }

            template<typename StateDescription, typename Measurement>
            static void measurementUpdate(KalmanFilter<StateDescription>& filter, Measurement measurement) {
                auto h = Measurement::predict(filter);
                auto H = Measurement::jacobian(filter);

                auto KA = filter.covariance * H.transpose();

                auto KI = (H * filter.covariance * H.transpose() + measurement.R).ldlt();


                auto l = filter.retrieve_lock();
                filter.state += KA * KI.solve(measurement.z - h);
                filter.covariance -= KA * KI.solve(H) * filter.covariance;
            }
        };
    }
}

#endif
