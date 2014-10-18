#pragma once
#ifndef SYS_MODELS_VWXQ_3D_HPP_
#define SYS_MODELS_VWXQ_3D_HPP_

#include <sys/math/filtering.hpp>
#include <sys/math/states.hpp>
#include <type_traits>

namespace sys {
    namespace math {
        namespace models {
            struct VWXQ_3D : public State<VWXQ_3D, 13> {
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

                /*
                 * Returns, for a given internal state number, the corresponding state number in
                 * the external state description.
                 */
                template<typename ExternalStates>
                constexpr static int statemap(const int state) {
                    return math::internal::StateMap{
                        ExternalStates::vx,
                        ExternalStates::vy,
                        ExternalStates::vz,
                        ExternalStates::wx,
                        ExternalStates::wy,
                        ExternalStates::wz,
                        ExternalStates::x,
                        ExternalStates::y,
                        ExternalStates::z,
                        ExternalStates::qx,
                        ExternalStates::qy,
                        ExternalStates::qz,
                        ExternalStates::qw}[state];
                }

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
