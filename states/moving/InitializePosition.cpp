#include <sys/states/moving/InitializePosition.hpp>
#include <sys/com/Stm.hpp>

namespace sys {
    namespace states {
        InitializePosition::InitializePosition(my_context ctx) : my_base(ctx)
        {
            context<Top>().stm.send(stm::ControlMessage{-200, 200});
        }

        InitializePosition::~InitializePosition() {
            context<Moving>().startingPosition = context<Moving>().planner.getClosestCheckpoint();
        }
    }
}
