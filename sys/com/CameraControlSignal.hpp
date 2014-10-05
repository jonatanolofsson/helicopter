#pragma once
#ifndef SYS_CAMERACONTROLSIGNALS_HPP_
#define SYS_CAMERACONTROLSIGNALS_HPP_

#include <Eigen/Core>
#include <sys/types.hpp>

namespace sys {
    using namespace Eigen;
    static const int numberOfCameraControlSignals = 2;
    typedef Matrix<Scalar, numberOfCameraControlSignals, 1> CameraControlSignal;
    namespace camera {
        static const int horizontal = 0;
        static const int vertical   = 0;
    }
}

#endif
