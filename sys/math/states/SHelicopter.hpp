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

                    qx = 6,
                    qy = 7,
                    qz = 8,
                    qw = 9,

                    x = 10,
                    y = 11,
                    z = 12,

                    windX = 13,
                    windY = 14,
                    windZ = 15,

                    th_a = 16,
                    th_b = 7,
                    th_c = 18,
                    th_tail = 19,
                    N = 20,

                    nofStates = 21
                };
                enum states {
                    position = x,
                    velocity = vx,
                    velocities = wx,
                    quaternion = qx,
                    rotationalVelocity = wx,
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
