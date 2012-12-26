#ifndef OS_SYS_OBSERVER_HPP_
#define OS_SYS_OBSERVER_HPP_

#include <sys/math/models/models.hpp>
#include <sys/math/filtering/ExtendedKalmanFilter.hpp>
#include <sys/math/filtering/KalmanFilter.hpp>
#include <Eigen/Core>

namespace sys {
    using namespace Eigen;
    typedef math::S6DOFQ StateDescription;
    typedef math::KalmanFilter<StateDescription> SystemState;
    typedef models::motion::ConstantVelocities  MotionModel;
    typedef math::ExtendedKalmanFilter filter;

    extern SystemState system_state;

    template<typename Sensor>
    struct Measurement : public Sensor {
        Matrix<typename Sensor::Scalar, Sensor::number_of_measurements, 1> z;
        Matrix<typename Sensor::Scalar, Sensor::number_of_measurements, Sensor::number_of_measurements> R;
    };

    template<typename Sensor>
    void measurementUpdate(const Measurement<Sensor> m) {
        filter::measurementUpdate<StateDescription,Measurement<Sensor>>(system_state, m);
    }
}

#endif
