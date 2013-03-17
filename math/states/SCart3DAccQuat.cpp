#include <sys/math/states/SCart3DAccQuat.hpp>

namespace sys {
    namespace math {
        namespace models {
            const int SCart3DAccQuat::orientation[4] = {SCart3DAccQuat::qx, SCart3DAccQuat::qy, SCart3DAccQuat::qz, SCart3DAccQuat::qw};
            const int SCart3DAccQuat::omega[3] = {SCart3DAccQuat::wx, SCart3DAccQuat::wy, SCart3DAccQuat::wz};
            const int SCart3DAccQuat::alpha[3] = {SCart3DAccQuat::alx, SCart3DAccQuat::aly, SCart3DAccQuat::alz};
        }
    }
}
