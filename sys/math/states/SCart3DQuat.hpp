#pragma once
#ifndef SYS_MODELS_SCART3DQUAT_HPP_
#define SYS_MODELS_SCART3DQUAT_HPP_

#include <sys/math/filtering.hpp>

namespace sys {
    namespace math {
        namespace models {
            template<typename S = Scalar, typename Model_ = void>
            struct SCart3DQuat {
                typedef Model_ Model;
                typedef S Scalar;

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
                    omega = wx
                };
                
                typedef math::internal::StateVector<Scalar, nofStates> StateVector;
                typedef messages::StateMessage<StateVector> StateMessage;

                static const int velocities[3];
                static const int omegas[3];
                static const int positions[3];
                static const int quaternions[4];

                constexpr static int statemap(const int state) {
                    return math::internal::StateMap({{
                            Model::ExternalStateDescription::velocities[0], 
                            Model::ExternalStateDescription::velocities[1], 
                            Model::ExternalStateDescription::velocities[2], 
                            Model::ExternalStateDescription::omegas[0], 
                            Model::ExternalStateDescription::omegas[1], 
                            Model::ExternalStateDescription::omegas[2], 
                            Model::ExternalStateDescription::positions[0], 
                            Model::ExternalStateDescription::positions[1], 
                            Model::ExternalStateDescription::positions[2], 
                            Model::ExternalStateDescription::quaternions[0], 
                            Model::ExternalStateDescription::quaternions[1], 
                            Model::ExternalStateDescription::quaternions[2], 
                            Model::ExternalStateDescription::quaternions[3]}})[state];
                }

                template<typename T>
                static void initialize(T& filter) {
                    auto l = filter.retrieve_lock();
                    filter.state.setZero();
                    filter.state[qw] = 1.0;

                    filter.covariance.setIdentity();
                }
            };
            template<typename S, typename Model_> const int SCart3DQuat<S, Model_>::velocities[3] = {SCart3DQuat::vx, SCart3DQuat::vy, SCart3DQuat::vz};
            template<typename S, typename Model_> const int SCart3DQuat<S, Model_>::omegas[3] = {SCart3DQuat::wx, SCart3DQuat::wy, SCart3DQuat::wz};
            template<typename S, typename Model_> const int SCart3DQuat<S, Model_>::positions[3] = {SCart3DQuat::x, SCart3DQuat::y, SCart3DQuat::z};
            template<typename S, typename Model_> const int SCart3DQuat<S, Model_>::quaternions[4] = {SCart3DQuat::qx, SCart3DQuat::qy, SCart3DQuat::qz, SCart3DQuat::qw};
        }
    }
}

#endif
