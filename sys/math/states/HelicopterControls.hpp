#pragma once

namespace sys {
    namespace math {
        namespace models {
            struct HelicopterControls : public State<HelicopterControls, 4> {
                enum state {
                    th_a = 0,
                    th_b = 1,
                    th_c = 2,
                    th_tail = 3
                };

                /*
                 * Returns, for a given internal state number, the corresponding state number in
                 * the external state description.
                 */
                template<typename ExternalStates>
                constexpr static int statemap(const int state) {
                    return math::internal::StateMap{
                        ExternalStates::th_a,
                        ExternalStates::th_b,
                        ExternalStates::th_c,
                        ExternalStates::th_tail}[state];
                }
            };
        }
    }
}

