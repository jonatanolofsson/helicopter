#pragma once

namespace sys {
    namespace math {
        namespace models {
            struct VWQi_3D : public State<VWQi_3D, 9> {
                enum control {
                    vx = 0,
                    vy = 1,
                    vz = 2,

                    wx = 3,
                    wy = 4,
                    wz = 5,

                    qx = 6,
                    qy = 7,
                    qz = 8
                };

                enum states {
                    velocity = vx,
                    omega = wx,
                    quaternion = qx
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
                        ExternalStates::qx,
                        ExternalStates::qy,
                        ExternalStates::qz}[state];
                }
            };
        }
    }
}

