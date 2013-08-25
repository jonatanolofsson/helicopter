#pragma once
#ifndef SYS_MODELS_SHELICOPTERCONTROL_HPP_
#define SYS_MODELS_SHELICOPTERCONTROL_HPP_

namespace sys {
    namespace math {
        namespace models {
            struct SHelicopterControl {
                enum state {
                    wx = 0,
                    wy = 1,
                    wz = 2,
                    
                    vx = 3,
                    vy = 4,
                    vz = 5,

                    th_tail = 6,
                    th_a = 7,
                    th_b = 8,
                    th_c = 9,
                    N = 10,

                    nofStates = 11
                };
                enum states {
                    velocity = vx,
                    rotationalVelocity = wx,
                };

                static const int omega[3];

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
