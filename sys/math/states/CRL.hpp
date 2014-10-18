#pragma once
#ifndef SYS_MODELS_CRL_HPP_
#define SYS_MODELS_CRL_HPP_

namespace sys {
    namespace math {
        namespace models {
            struct CRL {
                enum control {
                    vr = 0,
                    vl = 1,

                    nofControls = 2
                };
            };
        }
    }
}

#endif
