#include <sys/Simulator.hpp>
#include <sys/simulator/implementation.hpp>

namespace sys {
    namespace simulator {
        template class Simulator<MotionModel, sensors::Gps>;
    }
}
