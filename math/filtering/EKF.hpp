#ifndef SYS_MATH_FILTERING_EKF_HPP_
#define SYS_MATH_FILTERING_EKF_HPP_

#include <sys/math/filtering/GaussianFilter.hpp>
#include <Eigen/Cholesky>

namespace sys {
    namespace math {
        struct EKF {
            template<typename MotionModel, typename T>
            static void timeUpdate(T& filter, typename T::Scalar dT) {
                auto l = filter.retrieve_lock();
                auto A = MotionModel::systemJacobian(filter);

                filter.state = MotionModel::predict(filter, dT);
                filter.covariance = A*filter.covariance*A.transpose() + MotionModel::template covariance<T>(dT);
            }

            template<typename GaussianFilter, typename Measurement>
            static void measurementUpdate(GaussianFilter& filter, const Measurement& measurement) {
                auto h = Measurement::Model::predict(filter);
                auto H = Measurement::Model::jacobian(filter);

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
