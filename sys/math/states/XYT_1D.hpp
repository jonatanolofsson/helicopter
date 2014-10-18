#pragma once
#ifndef SYS_MODELS_XYT_1D_HPP_
#define SYS_MODELS_XYT_1D_HPP_

namespace sys {
    namespace math {
        namespace models {
            struct XYT_1D : public State<XYT_1D, 3> {
                enum state {
                    x = 0,
                    y = 1,
                    th = 2
                };
                enum states {
                    position = x,
                    angle = th
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
                        ExternalStates::th}[state];
                }

                template<typename T>
                static void initialize(T& filter) {
                    auto l = filter.retrieve_lock();
                    filter.state.setZero();
                }
            };
        }
    }
}

#endif
