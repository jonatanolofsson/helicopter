#pragma once
#ifndef SYS_SETTINGS_HPP_
#define SYS_SETTINGS_HPP_

#include <sys/types.hpp>

namespace sys {
    namespace settings {
        static const Scalar systemFrequency     = 100; ///< System frequency, in Hz
        static const Scalar dT                  = 1/systemFrequency; ///< System steptime
    }
}

#endif
