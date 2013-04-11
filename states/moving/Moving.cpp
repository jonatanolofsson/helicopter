#include <sys/states/moving/Moving.hpp>

namespace sys {
    namespace states {
        Moving::Moving(my_context ctx)
        : my_base(ctx)
        , actuator(context<Top>().stm)
        {}
    }
}
