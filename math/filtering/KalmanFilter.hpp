#ifndef SYS_MATH_FILTERING_KALMAN_FILTER_HPP_
#define SYS_MATH_FILTERING_KALMAN_FILTER_HPP_

#include <sys/math/filtering/types.hpp>
#include <os/mem/ProtectedData.hpp>

namespace sys {
    namespace math {
        template<typename T, typename S = Scalar>
        struct KalmanFilter : public os::ProtectedClass {
            typedef S Scalar;
            typedef typename Covariance<Scalar, T::number_of_states>::type CovarianceMatrix;
            typedef typename StateVector<Scalar, T::number_of_states>::type States;
            typedef typename StateVector<Scalar, T::number_of_controls>::type Controls;
            typedef T StateDescription;
            CovarianceMatrix covariance;
            States state;
            Controls controls;
            Scalar dT;

            explicit KalmanFilter(Scalar dT_) : dT(dT_) {
                state.setZero();
                covariance.setZero();
            }

            KalmanFilter<T,S>(const KalmanFilter<T,S>& f)
                : covariance(f.covariance)
                , state(f.state)
                , controls(f.controls)
                , dT(f.dT)
            {}

            const KalmanFilter<T,S> operator+(const States& s) const {
                KalmanFilter<T,S> r(*this);
                r.state += s;
                return r;
            }

            const KalmanFilter<T,S> operator-(const States& s) const {
                KalmanFilter<T,S> r(*this);
                r.state -= s;
                return r;
            }
        };
    }
}

#endif
