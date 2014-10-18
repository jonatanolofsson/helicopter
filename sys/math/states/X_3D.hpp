#pragma once
#ifndef SYS_MODELS_X_3D_HPP_
#define SYS_MODELS_X_3D_HPP_
#include <sys/math/states.hpp>

namespace sys {
    namespace math {
        namespace models {
            struct X_3D : public State<X_3D, 3> {
                enum state {
                    x = 0,
                    y = 1,
                    z = 2
                };

                /*
                 * Returns, for a given internal state number, the corresponding state number in
                 * the external state description.
                 */
                template<typename ExternalStates>
                constexpr static int statemap(const int state) {
                    return math::internal::StateMap{
                        ExternalStates::x,
                        ExternalStates::y,
                        ExternalStates::z}[state];
                }
            };
        }
    }
}

#endif
