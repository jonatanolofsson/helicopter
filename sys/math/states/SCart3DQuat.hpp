#pragma once
#ifndef SYS_MODELS_SCART3DQUAT_HPP_
#define SYS_MODELS_SCART3DQUAT_HPP_

#include <sys/math/filtering.hpp>
#include <sys/math/states.hpp>
#include <type_traits>

namespace sys {
    namespace math {
        namespace models {
            struct SCart3DQuat : public State<SCart3DQuat, 13> {
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
                };
                enum states {
                    position = x,
                    velocity = vx,
                    quaternion = qx,
                    omega = wx
                };

                static const int velocities[3];
                static const int omegas[3];
                static const int positions[3];
                static const int quaternions[4];

                /*
                 * Returns, for a given internal state number, the corresponding state number in
                 * the external state description.
                 */
                template<typename ExternalStates>
                constexpr static int statemap(const int state) {
                    return math::internal::StateMap{
                            ExternalStates::velocities[0],
                            ExternalStates::velocities[1],
                            ExternalStates::velocities[2],
                            ExternalStates::omegas[0],
                            ExternalStates::omegas[1],
                            ExternalStates::omegas[2],
                            ExternalStates::positions[0],
                            ExternalStates::positions[1],
                            ExternalStates::positions[2],
                            ExternalStates::quaternions[0],
                            ExternalStates::quaternions[1],
                            ExternalStates::quaternions[2],
                            ExternalStates::quaternions[3]}[state];
                }

                template<typename T>
                static void initialize(T& filter) {
                    auto l = filter.retrieve_lock();
                    filter.state.setZero();
                    filter.state[qw] = 1.0;

                    filter.covariance.setIdentity();
                }
            };
            const int SCart3DQuat::velocities[3] = {SCart3DQuat::vx, SCart3DQuat::vy, SCart3DQuat::vz};
            const int SCart3DQuat::omegas[3] = {SCart3DQuat::wx, SCart3DQuat::wy, SCart3DQuat::wz};
            const int SCart3DQuat::positions[3] = {SCart3DQuat::x, SCart3DQuat::y, SCart3DQuat::z};
            const int SCart3DQuat::quaternions[4] = {SCart3DQuat::qx, SCart3DQuat::qy, SCart3DQuat::qz, SCart3DQuat::qw};
        }
    }
}

#endif
