#pragma once

#include <Eigen/Core>
#include <sys/types.hpp>

namespace sys {
    using namespace Eigen;
    static const int numberOfMotionControlSignals = 4;
    typedef Matrix<Scalar, numberOfMotionControlSignals, 1> MotionControlSignal;
    namespace control {
        static const int servo[3] = {0,1,2};
        static const int rpm = 3;
    }
}

