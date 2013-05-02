#include <sys/globalplanner/API.hpp>
#include <sys/globalplanner/GlobalPlanner.hpp>

namespace sys {
    namespace globalplanner {
        GlobalPlanner::GlobalPlanner()
        : d_position(&Self::checkPosition, this)
        {}

        void GlobalPlanner::checkPosition(const observer::SystemState x) {
            using namespace math;
            if(distanceBetween(Position(x), checkpoint) < settings.
        }
    }
}
