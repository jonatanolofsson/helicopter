#pragma once
#ifndef SYS_MODELS_CHELICOPTER_HPP_
#define SYS_MODELS_CHELICOPTER_HPP_

namespace sys {
    namespace math {
        namespace models {
            struct CHelicopter {
                enum control {
                    th_tail = 0,
                    th_a = 1,
                    th_b = 2,
                    th_c = 3,
                    N = 4,
                    
                    nofControls = 5
                };
            };
        }
    }
}

#endif
