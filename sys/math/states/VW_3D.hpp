#pragma once
#ifndef SYS_MODELS_VW_3D_HPP_
#define SYS_MODELS_VW_3D_HPP_

namespace sys {
    namespace math {
        namespace models {
            struct VW_3D : public State<VW_3D, 6> {
                enum control {
                    vx = 0,
                    vy = 1,
                    vz = 2,

                    wx = 3,
                    wy = 4,
                    wz = 5
                };

                enum states {
                    velocity = vx,
                    rotationalVelocity = wx
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
                        ExternalStates::wz}[state];
                }
            };
        }
    }
}

#endif
