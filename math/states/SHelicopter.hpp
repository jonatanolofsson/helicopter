#pragma once
#ifndef SYS_MODELS_SHELICOPTER_HPP_
#define SYS_MODELS_SHELICOPTER_HPP_

namespace sys {
    namespace math {
        namespace models {
            struct SHelicopter {
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

                    qx = 11,
                    qy = 12,
                    qz = 13,
                    qw = 14,

                    x = 15,
                    y = 16,
                    z = 17,

                    wbx = 18,
                    wby = 19,
                    wbz = 20,

                    windX = 21,
                    windY = 22,
                    windZ = 23,

                    nofStates = 24
                };
                enum states {
                    position = x,
                    velocity = vx,
                    quaternion = qx,
                    rotationalVelocity = wx,
                    controlled = th_tail,
                    nonControlled = wx
                };

                static const int orientation[4];
                static const int omega[3];

                template<typename T>
                static void initializeState(T& state) {
                    state.setZero();
                    state[qw] = 1.0;
                }

                template<typename T>
                static void initialize(T& filter) {
                    auto l = filter.retrieve_lock();
                    initializeState(filter.state);
                    filter.covariance.setIdentity();
                }
            };
        }
    }
}

#endif
