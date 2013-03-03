#include <sys/math/models/SCart3DQuat.hpp>

namespace sys {
    namespace math {
        namespace models {
            const int SCart3DQuat::omega[3] = {SCart3DQuat::wx, SCart3DQuat::wy, SCart3DQuat::wz};
            const int SCart3DQuat::orientation[4] = {SCart3DQuat::qx, SCart3DQuat::qy, SCart3DQuat::qz, SCart3DQuat::qw};
        }
    }
}
