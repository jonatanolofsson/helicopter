#pragma once
#ifndef SYS_MODELS_C6VW_HPP_
#define SYS_MODELS_C6VW_HPP_

namespace sys {
    namespace math {
        namespace models {
            struct C6VW {
                enum control {
                    vx = 0,
                    vy = 1,
                    vz = 2,

                    wx = 3,
                    wy = 4,
                    wz = 5,

                    nofControls = 6
                };
                enum states {
                    velocity = vx,
                    rotationalVelocity = wx
                };

                static const int omega[3];
            };
        }
    }
}

#endif
