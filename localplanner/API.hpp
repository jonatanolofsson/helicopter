#pragma once
#ifndef SYS_LOCALPLANNER_API_HPP_
#define SYS_LOCALPLANNER_API_HPP_

#include <sys/observer/API.hpp>

namespace sys {
    namespace localplanner {
        typedef observer::StateDescription StateDescription;
        static const Scalar togglingDistance        = 5e-2;
        static const Scalar baseVelocity            = 2e-1;
        static const Scalar rotateFirstLimit        = 2e-1;

        typedef math::astar::Graph<4, 2, math::astar::CartesianPosition2d> Graph;
        typedef Graph::NodeType                     Checkpoint;
    }
}

#include <sys/localplanner/LocalPlanner.hpp>

namespace sys {
    typedef localplanner::LocalPlanner LocalPlanner;
}

#endif
