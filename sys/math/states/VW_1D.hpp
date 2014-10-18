#pragma once
#ifndef SYS_MODELS_VW_1D_HPP_
#define SYS_MODELS_VW_1D_HPP_

namespace sys {
    namespace math {
        namespace models {
            struct VW_1D : public State<VW_1D, 2> {
                enum state {
                    v = 0,
                    w = 1
                };

                /*
                 * Returns, for a given internal state number, the corresponding state number in
                 * the external state description.
                 */
                template<typename ExternalStates>
                constexpr static int statemap(const int state) {
                    return math::internal::StateMap{
                        ExternalStates::v,
                        ExternalStates::w}[state];
                }
            };
        }
    }
}

#endif
