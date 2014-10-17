#pragma once
#ifndef SYS_MODELS_SCART3DQUAT_HPP_
#define SYS_MODELS_SCART3DQUAT_HPP_

#include <sys/math/filtering.hpp>
#include <sys/com/StateMessage.hpp>
#include <type_traits>

namespace sys {
    namespace math {
        namespace models {
            template<typename Model_ = void>
            struct SCart3DQuat {
                typedef Model_ Model;

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

                /*
                 * Returns, for a given internal state number, the corresponding state number in
                 * the external state description.
                 */
                template<typename ExternalStateDescription = Model::ExternalStateDescription>
                constexpr static int statemap(const int state) {
                    return math::internal::StateMap{
                            ExternalStateDescription::velocities[0],
                            ExternalStateDescription::velocities[1],
                            ExternalStateDescription::velocities[2],
                            ExternalStateDescription::omegas[0],
                            ExternalStateDescription::omegas[1],
                            ExternalStateDescription::omegas[2],
                            ExternalStateDescription::positions[0],
                            ExternalStateDescription::positions[1],
                            ExternalStateDescription::positions[2],
                            ExternalStateDescription::quaternions[0],
                            ExternalStateDescription::quaternions[1],
                            ExternalStateDescription::quaternions[2],
                            ExternalStateDescription::quaternions[3]}[state];
                }

                template<typename ExternalStateDescription = Model::ExternalStateDescription>
                StateVector extract(const ExternalStateDescription::StateVector& x) {
                    StateVector r;
                    for(int i = 0; i < nofStates; ++i) {
                        r(i) = x(statemap<ExternalStateDescription>(i));
                    }
                    return r;
                }

                template<typename T>
                static void initialize(T& filter) {
                    auto l = filter.retrieve_lock();
                    filter.state.setZero();
                    filter.state[qw] = 1.0;

                    filter.covariance.setIdentity();
                }
            };
            template<typename Model_> const int SCart3DQuat<Model_>::velocities[3] = {SCart3DQuat<Model_>::vxCart3DQuat<Model_>::vyCart3DQuat<Model_>::vz};
            template<typename Model_> const int SCart3DQuat<Model_>::omegas[3] = {SCart3DQuat<Model_>::wxCart3DQuat<Model_>::wyCart3DQuat<Model_>::wz};
            template<typename Model_> const int SCart3DQuat<Model_>::positions[3] = {SCart3DQuat<Model_>::xCart3DQuat<Model_>::yCart3DQuat<Model_>::z};
            template<typename Model_> const int SCart3DQuat<Model_>::quaternions[4] = {SCart3DQuat<Model_>::qxCart3DQuat<Model_>::qyCart3DQuat<Model_>::qzCart3DQuat<Model_>::qw};
        }
    }
}

#endif
