#pragma once
#ifndef SYS_MODELS_SCART3DACCQUAT_HPP_
#define SYS_MODELS_SCART3DACCQUAT_HPP_

namespace sys {
    namespace math {
        namespace models {
            struct SCart3DAccQuat {
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

                    ax = 9,
                    ay = 10,
                    az = 11,

                    alx = 12,
                    aly = 13,
                    alz = 14,

                    qx = 15,
                    qy = 16,
                    qz = 17,
                    qw = 18,

                    nofStates = 19
                };
                enum states {
                    position = x,
                    velocity = vx,
                    acceleration = ax,
                    quaternion = qx,
                    rotational_velocity = wx,
                    rotational_acceleration = alx
                };

                static const int orientation[4];
                static const int omega[3];
                static const int alpha[3];

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
