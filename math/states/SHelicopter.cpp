#include <sys/math/states/SHelicopter.hpp>

namespace sys {
    namespace math {
        namespace models {
            const int SHelicopter::orientation[4] = {SHelicopter::qx, SHelicopter::qy, SHelicopter::qz, SHelicopter::qw};
            const int SHelicopter::omega[3] = {SHelicopter::wx, SHelicopter::wy, SHelicopter::wz};
        }
    }
}
