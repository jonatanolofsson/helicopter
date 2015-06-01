#pragma once

namespace sys {
    namespace math {
        namespace models {
            struct HelicopterStates : public State<HelicopterStates, 21> {
                enum state {
                    vx = 0,
                    vy = 1,
                    vz = 2,

                    wx = 3,
                    wy = 4,
                    wz = 5,

                    x = 6,
                    y = 7,
                    z = 8,

                    qx = 9,
                    qy = 10,
                    qz = 11,
                    qw = 12,

                    windX = 13,
                    windY = 14,
                    windZ = 15,

                    th_a = 16,
                    th_b = 17,
                    th_c = 18,
                    th_tail = 19,
                    N = 20
                };
                enum states {
                    position = x,
                    omega = wx,
                    velocity = vx,
                    quaternion = qx,
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
                        ExternalStates::x,
                        ExternalStates::y,
                        ExternalStates::z,
                        ExternalStates::qx,
                        ExternalStates::qy,
                        ExternalStates::qz,
                        ExternalStates::qw,
                        ExternalStates::windX,
                        ExternalStates::windY,
                        ExternalStates::windZ,
                        ExternalStates::th_a,
                        ExternalStates::th_b,
                        ExternalStates::th_c,
                        ExternalStates::th_tail,
                        ExternalStates::N}[state];
                }

                template<typename T>
                static void initializeState(T& state) {
                    state.setZero();
                    state[qw] = 1.0;
                }

                template<typename T>
                static void initialize(T& filter) {
                    auto l = filter.retrieve_lock();
                    initializeState(filter.state);
                    filter.covariance.setIdentity();
                }
            };
        }
    }
}

