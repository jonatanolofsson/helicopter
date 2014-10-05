#pragma once
#ifndef SYS_MODELS_CVW_HPP_
#define SYS_MODELS_CVW_HPP_

namespace sys {
    namespace math {
        namespace models {
            struct CVW {
                enum control {
                    v = 0,
                    w = 1,

                    nofControls = 2
                };
            };
        }
    }
}

#endif
