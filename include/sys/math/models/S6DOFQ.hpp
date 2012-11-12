#ifndef SYS_MODELS_S6DOFQ_HPP_
#define SYS_MODELS_S6DOFQ_HPP_

namespace sys {
    namespace state_description {
        struct S6DOFQ {
            enum state {
                x = 0,
                y = 1,
                z = 2,

                vx = 3,
                vy = 4,
                vz = 5,

                qx = 6,
                qy = 7,
                qz = 8,
                qw = 9,

                wx = 10,
                wy = 11,
                wz = 12,

                number_of_states = 13
            };
            enum states {
                position = x,
                velocity = vx,
                quaternion = qx,
                rotational_velocity = wx
            };

            static const int omega[3];
            static const int orientation[4];
            enum control {
                number_of_controls = 0
            };

            template<typename T>
            static void initialize(T& filter) {
                filter.lock();
                    filter.state.setZero();
                    filter.state[qw] = 1.0;
                filter.unlock();
            }
        };

        const int S6DOFQ::omega[3] = {S6DOFQ::wx, S6DOFQ::wy, S6DOFQ::wz};
        const int S6DOFQ::orientation[4] = {S6DOFQ::qx, S6DOFQ::qy, S6DOFQ::qz, S6DOFQ::qw};
    }
}

#endif
