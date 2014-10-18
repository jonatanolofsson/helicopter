#include <sys/Simulator.hpp>
#include <sys/simulator/implementation.hpp>
#include <os/com/getSignal.hpp>

namespace sys {
    namespace simulator {
        template class Simulator<MotionModel, observer::sensors::Gps>;
    }
}
