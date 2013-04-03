#pragma once
#ifndef SYS_MODELS_S2DPOSE_HPP_
#define SYS_MODELS_S2DPOSE_HPP_

namespace sys {
    namespace math {
        namespace models {
            struct S2DPose {
                enum state {
                    x = 0,
                    y = 1,
                    th = 2,

                    nofStates = 3
                };
                enum states {
                    position = x,
                    angle = th,
                };

                template<typename T>
                static void initialize(T& filter) {
                    auto l = filter.retrieve_lock();
                    filter.state.setZero();
                }
            };
        }
    }
}

#endif
