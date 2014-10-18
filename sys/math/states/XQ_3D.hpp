#pragma once
#ifndef SYS_MODELS_XQ_3D_HPP_
#define SYS_MODELS_XQ_3D_HPP_

#include <sys/math/filtering.hpp>
#include <sys/math/states.hpp>
#include <type_traits>

namespace sys {
    namespace math {
        namespace models {
            struct XQ_3D : public State<XQ_3D, 7> {
                enum state {
                    x = 0,
                    y = 1,
                    z = 2,

                    qx = 3,
                    qy = 4,
                    qz = 5,
                    qw = 6,
                };
                enum states {
                    position = x,
                    quaternion = qx,
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
                        ExternalStates::z,
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
