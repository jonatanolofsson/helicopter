#pragma once
#ifndef SYS_MODELS_XYTVWA_1D_HPP_
#define SYS_MODELS_XYTVWA_1D_HPP_

namespace sys {
    namespace math {
        namespace models {
            struct XYTVWA_1D : public State<XYTVWA_1D, 6> {
                enum state {
                    x = 0,
                    y = 1,
                    th = 2,
                    v = 3,
                    w = 4,
                    a = 5
                };
                enum states {
                    position = x,
                    velocity = v,
                    acceleration = a,
                    angle = th,
                    angular_velocity = w
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
                        ExternalStates::th,
                        ExternalStates::v,
                        ExternalStates::w,
                        ExternalStates::a}[state];
                }

                template<typename T>
                static void initialize(T& filter) {
                    auto l = filter.retrieve_lock();
                    filter.state.setZero();

                    filter.covariance.setIdentity();
                }
            };
        }
    }
}

#endif
