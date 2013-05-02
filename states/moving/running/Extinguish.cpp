#include <sys/states/moving/running/Extinguish.hpp>

namespace sys {
    namespace states {
        Extinguish::Extinguish() {
            context<Top>().stm.send(stm::ControlMessage{0,0});
        }

        Extinguish::~Extinguish() {
        }
    }
}
