#pragma once
#ifndef SYS_MODELS_VWXALQ_3D_HPP_
#define SYS_MODELS_VWXALQ_3D_HPP_

namespace sys {
    namespace math {
        namespace models {
            struct VWXALQ_3D : public State<VWXALQ_3D, 19>  {
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
                };

                enum states {
                    position = x,
                    velocity = vx,
                    acceleration = ax,
                    quaternion = qx,
                    rotational_velocity = wx,
                    rotational_acceleration = alx
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
                        ExternalStates::ax,
                        ExternalStates::ay,
                        ExternalStates::az,
                        ExternalStates::alx,
                        ExternalStates::aly,
                        ExternalStates::alz,
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
