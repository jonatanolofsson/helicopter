#include <sys/simulator/API.hpp>
#include <sys/simulator/implementation.hpp>

namespace sys {
    namespace simulator {
        template class Simulator<MotionModel, FilterState>;
    }
}
