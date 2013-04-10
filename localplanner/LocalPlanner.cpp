#include <sys/localplanner/API.hpp>
#include <sys/motioncontrol/API.hpp>
#include <sys/states/events.hpp>
#include <sys/states/API.hpp>
#include <os/com/getSignal.hpp>
#include <iostream>

namespace sys {
    namespace localplanner {
        static const Graph graph{{
            {1, {0.0,0.0},      {2, 4}},
            {2, {1.0,0.0},      {1, 3}},
            {3, {1.5,0.5},      {2, 4}},
            {4, {0.0,1.0},      {1, 3}},
        }};

        LocalPlanner::LocalPlanner()
        : objectives{
                &graph.nodes[0],
                &graph.nodes[1],
            }
        , d_reference(&Self::yieldReference, this)
        , closestCheckpoint(nullptr)
        {}

        void LocalPlanner::yieldReference(const observer::SystemState x) {
            if((checkpoints.size() == 0) && (objectives.size() == 0)) {
                return;
            }

            typedef StateDescription states;
            std::cout << "Localplanner: Yield reference" << std::endl;

            Eigen::Vector2f position = x.segment<2>(states::position);

            auto cpPosition = Eigen::Vector2f(checkpoints.top()->position.x, checkpoints.top()->position.y);
            math::astar::CartesianPosition2d pos{x[states::x], x[states::y]};
            closestCheckpoint = graph.closest(pos);

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
            ref << ((angle < rotateFirstLimit) ? baseVelocity : 0.0), math::toPi(angle - x[states::th]);
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
