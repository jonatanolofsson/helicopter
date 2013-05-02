#pragma once
#ifndef SYS_MODELS_SCART3DQUAT_HPP_
#define SYS_MODELS_SCART3DQUAT_HPP_

namespace sys {
    namespace math {
        namespace models {
            struct SCart3DQuat {
                enum state {
                    vx = 0,
                    vy = 1,
                    vz = 2,

                    wx = 3,
                    wy = 4,
                    wz = 5,

                    x = 6,
                    y = 7,
                    z = 8,

                    qx = 9,
                    qy = 10,
                    qz = 11,
                    qw = 12,

                    nofStates = 13
                };
                enum states {
                    position = x,
                    velocity = vx,
                    quaternion = qx,
                    rotational_velocity = wx
                };

                static const int omega[3];
                static const int orientation[4];

                template<typename T>
                static void initialize(T& filter) {
                    auto l = filter.retrieve_lock();
                    filter.state.setZero();
                    filter.state[qw] = 1.0;

                    filter.covariance.setIdentity();
                }
            };
        }
    }
}

#endif
