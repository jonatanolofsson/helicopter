#ifndef SYS_MATH_FILTERING_EXTENDED_KALMAN_FILTER_HPP_
#define SYS_MATH_FILTERING_EXTENDED_KALMAN_FILTER_HPP_

#include <sys/math/filtering/KalmanFilter.hpp>

namespace sys {
    namespace math {
        template<typename MotionModel>
        struct ExtendedKalmanFilter {
            template<typename T>
            static void timeUpdate(T& filter) {
                filter.lock();
                    auto A = MotionModel::systemJacobian(filter);

                    filter.state = MotionModel::predict(filter);
                    filter.covariance = A*filter.covariance*A.transpose() + MotionModel::template Q<T>();
                filter.unlock();
            }

            template<typename StateDescription, typename Measurement>
            static void measurementUpdate(const KalmanFilter<StateDescription>& filter, Measurement measurement) {
                filter.lock();
                    auto h = Measurement::predict(filter);
                    auto H = Measurement::jacobian(filter);

                    auto KA = filter.covariance * H.transpose();
                    auto KI = (H * filter.covariance * H.transpose() + measurement.R).ldlt();
                    filter.state += KA * KI.solve(measurement.z - h);
                    filter.covariance -= KA * KI.solve(H) * filter.covariance;
                filter.unlock();
            }
        };
    }
}

#endif
