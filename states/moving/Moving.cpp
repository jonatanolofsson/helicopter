#include <sys/states/moving/Moving.hpp>

namespace sys {
    namespace states {
        Moving::Moving()
        : actuator(context<Top>().maple)
        {}
    }
}
