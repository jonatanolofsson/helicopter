#pragma once
#ifndef SYS_MODELS_SCT2D_HPP_
#define SYS_MODELS_SCT2D_HPP_

namespace sys {
    namespace math {
        namespace models {
            struct SCT2D {
                enum state {
                    x = 0,
                    y = 1,
                    th = 2,
                    v = 3,
                    w = 4,
                    a = 5,

                    nofStates = 6
                };
                enum states {
                    position = x,
                    velocity = v,
                    acceleration = a,
                    angle = th,
                    angular_velocity = w
                };

                template<typename T>
                static void initialize(T& filter) {
                    auto l = filter.retrieve_lock();
                    filter.state.setZero();

                    filter.covariance.setIdentity();
                }
            };
        }
    }
}

#endif
