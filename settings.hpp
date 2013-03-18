#pragma once
#ifndef SYS_SETTINGS_HPP_
#define SYS_SETTINGS_HPP_

#include <os/types.hpp>

namespace sys {
    namespace settings {
        using os::Scalar;

        static const Scalar system_frequency    = 100; ///< System frequency, in Hz
        static const Scalar dT                  = 1/system_frequency; ///< System steptime
    }
}

#endif
