#pragma once
#ifndef SYS_MODELS_V_3D_HPP_
#define SYS_MODELS_V_3D_HPP_

namespace sys {
    namespace math {
        namespace models {
            struct V_3D : public State<V_3D, 3> {
                enum state {
                    vx = 0,
                    vy = 1,
                    vz = 2
                };
                enum states {
                    velocity = vx
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
                        ExternalStates::vz}[state];
                }
            };
        }
    }
}

#endif
