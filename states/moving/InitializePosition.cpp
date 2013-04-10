#include <sys/states/moving/InitializePosition.hpp>
#include <sys/com/Stm.hpp>

namespace sys {
    namespace states {
        InitializePosition::~InitializePosition() {
            context<Moving>().startingPosition = context<Moving>().planner.getClosestCheckpoint();
        }
        InitializePosition::InitializePosition()
        {
            context<Top>().stm.send(stm::ControlMessage{-200, 200});
        }
    }
}
