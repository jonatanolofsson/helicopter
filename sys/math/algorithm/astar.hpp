#pragma once
#ifndef SYS_MATH_ASTAR_HPP_
#define SYS_MATH_ASTAR_HPP_

#include <list>
#include <stack>
#include <set>
#include <algorithm>

namespace sys {
    namespace math {
        namespace astar {
            template<int maxNofNeighbours, typename Position>
            struct Node {
                typedef Node<maxNofNeighbours, Position> Self;
                unsigned int id;
                Position position;
                unsigned int neighbours[maxNofNeighbours];
            };


            template<int nofNodes_, int maxNofNeighbours_, typename NodeData_>
            struct Graph {
                typedef NodeData_ NodeData;
                typedef Node<maxNofNeighbours_, NodeData> NodeType;
                NodeType nodes[nofNodes_];

                static const int nofNodes = nofNodes_;
                static const int maxNofNeighbours = maxNofNeighbours_;

                const NodeType* closest(const NodeData a) const {
                    const NodeType* c = &nodes[0];
                    auto maxDistance = 1e20;
                    for(const auto& n : nodes) {
                        auto d = pDistance(a, n.position);
                        if(d < maxDistance) {
                            maxDistance = d;
                            c = &n;
                        }
                    }

                    return c;
                }
            };

            struct CartesianPosition2d {
                float x,y;
            };

            template<typename GraphType, float(*heuristic)(const typename GraphType::NodeType&, const typename GraphType::NodeType&), float(*edgecost)(const typename GraphType::NodeType&, const typename GraphType::NodeType&) = heuristic>
            std::stack<const typename GraphType::NodeType*> astar(const GraphType& graph, const unsigned int from, const unsigned int to) {
                float gscore[GraphType::nofNodes] = {0};
                float fscore[GraphType::nofNodes] = {0};
                float came_from[GraphType::nofNodes] = {0};
                fscore[from-1] = heuristic(graph.nodes[from-1], graph.nodes[to-1]);

                auto& destination = graph.nodes[to-1];

                std::list<unsigned int> open;
                std::set<unsigned int> closed;
                std::stack<const typename GraphType::NodeType*> path;

                open.push_back(from);

                while(!open.empty()) {
                    unsigned int currentIndex = open.front()-1;
                    auto& current = graph.nodes[currentIndex];
                    if(current.id == to) goto reconstructPath;
                    open.pop_front();
                    closed.insert(current.id);

                    for(int i = 0; i < GraphType::maxNofNeighbours; ++i) {
                        unsigned int neighbour = current.neighbours[i];
                        if(!neighbour) break;
                        unsigned int neighbourIndex = neighbour - 1;
                        float new_gscore = gscore[currentIndex] + edgecost(current, graph.nodes[neighbourIndex]);
                        gscore[neighbourIndex] = new_gscore;
                        if((closed.find(neighbour) != closed.end()) && (gscore[neighbourIndex] >= new_gscore)) continue;

                        bool neighbourNotInOpenSet = (std::find(open.begin(), open.end(), neighbour) == open.end());
                        if(neighbourNotInOpenSet || (new_gscore < gscore[neighbourIndex])) {
                            came_from[neighbourIndex] = current.id;
                            gscore[neighbourIndex] = new_gscore;
                            fscore[neighbourIndex] = new_gscore + heuristic(current, destination);
                            if(neighbourNotInOpenSet) {
                                open.push_back(neighbour);
                            }
                        }
                    }
                    open.sort([&](const int a, const int b) { return fscore[a-1] < fscore[b-1]; });
                }

                // No path was found from a to b
                return path;

            reconstructPath:
                unsigned int l = to;
                while(l && l != from) {
                    path.push(&graph.nodes[l-1]);
                    l = came_from[l-1];
                }

                return path;
            }

            template<typename D>
            float pDistance(const D& a, const D& b) {
                auto dx = b.x - a.x;
                auto dy = b.y - a.y;

                return dx*dx + dy*dy;
            }

            template<typename GraphType>
            float distance(const typename GraphType::NodeType& a, const typename GraphType::NodeType& b) {
                auto dx = b.position.x - a.position.x;
                auto dy = b.position.y - a.position.y;

                return dx*dx + dy*dy;
            }
        }
    }
}

#endif
