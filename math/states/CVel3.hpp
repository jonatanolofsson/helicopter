#pragma once
#ifndef SYS_MODELS_CVEL3_HPP_
#define SYS_MODELS_CVEL3_HPP_

namespace sys {
    namespace math {
        namespace models {
            struct CVel3 {
                enum control {
                    vx = 0,
                    vy = 1,
                    vz = 2,

                    nofControls = 3
                };
                enum states {
                    velocity = vx
                };

                template<typename T>
                static void initialize(T& filter) {
                    auto l = filter.retrieve_lock();
                    filter.control.setZero();
                }
            };
        }
    }
}

#endif
