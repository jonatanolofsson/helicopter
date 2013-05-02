#include <sys/states/moving/running/Return.hpp>

namespace sys {
    namespace states {
        Return::Return() {
            context<Moving>().planner.clearObjectives();
            context<Moving>().planner.addObjective(context<Moving>().startingPosition);
        }
    }
}
