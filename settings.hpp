#pragma once
#ifndef SYS_SETTINGS_HPP_
#define SYS_SETTINGS_HPP_

#include <os/types.hpp>

namespace sys {
    namespace settings {
        using os::Scalar;

        static const Scalar systemFrequency     = 100; ///< System frequency, in Hz
        static const Scalar dT                  = 1/systemFrequency; ///< System steptime

        static const Scalar g                   = 9.82331;
    }
}

#endif
