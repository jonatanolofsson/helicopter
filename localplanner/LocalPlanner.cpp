#include <sys/localplanner/API.hpp>
#include <sys/motioncontrol/API.hpp>
#include <sys/states/events.hpp>
#include <sys/states/API.hpp>
#include <os/com/getSignal.hpp>
#include <iostream>

namespace sys {
    namespace localplanner {
        static const Graph graph{{
            {1,     {0.35,0.20},      {2, 4}},
            {2,     {0.94,0.20},      {1, 5}},
            {3,     {1.77,0.20},      {6}},
            {4,     {0.35,0.80},      {1}},
            {5,     {0.94,0.80},      {2, 6, 9}},
            {6,     {1.77,0.80},      {3, 5}},
            {7,     {0.20,1.30},      {8, 13}},
            {8,     {0.90,1.30},      {7, 9, 14}},
            {9,     {0.94,1.20},      {5, 8, 10}},
            {10,    {1.35,1.20},      {9, 11, 12}},
            {11,    {2.20,1.20},      {10, 15}},
            {12,    {1.35,1.62},      {10}},
            {13,    {0.20,2.00},      {7}},
            {14,    {0.90,2.20},      {8, 15}},
            {15,    {2.20,2.20},      {11, 14}}
        }};

        LocalPlanner::LocalPlanner()
        : objectives{
                &graph.nodes[0],
                &graph.nodes[5],
                &graph.nodes[6],
                &graph.nodes[11],
                &graph.nodes[14],
            }
        , d_reference(&Self::yieldReference, this)
        , closestCheckpoint(nullptr)
        {}

        void LocalPlanner::yieldReference(const observer::SystemState x) {
            typedef StateDescription states;
            std::cout << "Localplanner: Yield reference" << std::endl;
            Eigen::Vector2f position = x.segment<2>(states::position);
            math::astar::CartesianPosition2d pos{x[states::x], x[states::y]};

            if(checkpoints.size() == 0) {
                if(objectives.size() == 0) {
                    postEvent(events::OutOfObjectives());
                    return;
                } else {
                    closestCheckpoint = graph.closest(pos);
                    objectives.sort(
                        [&](const Checkpoint* a, const Checkpoint* b) -> bool {
                            return math::astar::pDistance(a->position, pos) < math::astar::pDistance(b->position, pos);
                        });
                    auto nextObjective = objectives.front();
                    objectives.pop_front();
                    checkpoints = math::astar::astar<decltype(graph), math::astar::distance<decltype(graph)>>(graph, closestCheckpoint->id, nextObjective->id);
                }
            }

            if(checkpoints.size() == 0) {
                postEvent(events::OutOfObjectives());
                return;
            }


            auto cpPosition = Eigen::Vector2f(checkpoints.top()->position.x, checkpoints.top()->position.y);
            if((position - cpPosition).norm() <= togglingDistance) {
                checkpoints.pop();
                if(checkpoints.size() == 0) {
                    if(objectives.size() > 0) {
                        objectives.sort(
                            [&](const Checkpoint* a, const Checkpoint* b) -> bool {
                                return math::astar::pDistance(a->position, pos) < math::astar::pDistance(b->position, pos);
                            });
                        auto nextObjective = objectives.front();
                        objectives.pop_front();
                        checkpoints = math::astar::astar<decltype(graph), math::astar::distance<decltype(graph)>>(graph, closestCheckpoint->id, nextObjective->id);
                    } else {
                        postEvent(events::OutOfObjectives());
                    }
                }
            }

            cpPosition = Eigen::Vector2f(checkpoints.top()->position.x, checkpoints.top()->position.y);
            Scalar angle = math::angleFromTo(x.segment<2>(states::position), cpPosition);
            static motioncontrol::Reference ref;
            ref << ((angle < rotateFirstLimit) ? baseVelocity : 0.0), math::toPi(angle - x[states::th]), math::toPi(angle);
            os::yield(ref);
        }

        void LocalPlanner::clearObjectives() {
            objectives.clear();
        }
        void LocalPlanner::addObjective(const Checkpoint* o) {
            objectives.push_back(o);
        }

        const Checkpoint* LocalPlanner::getClosestCheckpoint() {
            return closestCheckpoint;
        }
    }
}
