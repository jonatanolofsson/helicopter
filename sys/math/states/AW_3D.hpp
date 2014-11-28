#pragma once

namespace sys {
    namespace math {
        namespace models {
            struct AW_3D : public State<AW_3D, 6> {
                enum control {
                    ax = 0,
                    ay = 1,
                    az = 2,

                    wx = 3,
                    wy = 4,
                    wz = 5
                };

                enum states {
                    acceleration = ax,
                    omega = wx
                };

                /*
                 * Returns, for a given internal state number, the corresponding state number in
                 * the external state description.
                 */
                template<typename ExternalStates>
                constexpr static int statemap(const int state) {
                    return math::internal::StateMap{
                        ExternalStates::ax,
                        ExternalStates::ay,
                        ExternalStates::az,
                        ExternalStates::wx,
                        ExternalStates::wy,
                        ExternalStates::wz}[state];
                }
            };
        }
    }
}

