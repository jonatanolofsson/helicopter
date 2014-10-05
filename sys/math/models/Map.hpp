#pragma once
#ifndef SYS_MATH_MODELS_MAP_HPP_
#define SYS_MATH_MODELS_MAP_HPP_

#include <limits>

namespace sys {
    namespace math {
        template<unsigned N>
        struct Map {
            struct Line {
                struct Coordinate {
                    Scalar x;
                    Scalar y;
                };
                Coordinate p1;
                Coordinate p2;
            };
            typedef Line Wall;

            Wall walls[N];

            template<typename ModelDescription>
            Scalar getIntersection(const typename ModelDescription::States& state) const {
                //~ std::cout << "Finding intersection for sensor state " << state.transpose() << std::endl;
                typedef typename ModelDescription::StateDescription states;
                typedef typename ModelDescription::Scalar Scalar;
                Scalar ix, minx, maxx;
                Scalar iy, miny, maxy;
                Scalar dist;
                Scalar shortestDist = std::numeric_limits<Scalar>::max();

                Line sensorLine = {{state[states::x], state[states::y]}, {state[states::x] + std::cos(state[states::th]), state[states::y] + std::sin(state[states::th])}};
                //~ std::cout << "with sensor line (" << sensorLine.p1.x << "," << sensorLine.p1.y << ") ->  (" << sensorLine.p2.x << "," << sensorLine.p2.y << ")" << std::endl;
                Line perpSensorLine = {{state[states::x], state[states::y]}, {sensorLine.p2.y - sensorLine.p1.y + sensorLine.p1.x, sensorLine.p1.x - sensorLine.p2.x + sensorLine.p1.y}};
                static const Scalar e = 1e-5;
                for(const Wall& wall : walls) {
                    ix = ((sensorLine.p1.x*sensorLine.p2.y-sensorLine.p1.y*sensorLine.p2.x)*(wall.p1.x-wall.p2.x)-(sensorLine.p1.x-sensorLine.p2.x)*(wall.p1.x*wall.p2.y-wall.p1.y*wall.p2.x))/((sensorLine.p1.x-sensorLine.p2.x)*(wall.p1.y-wall.p2.y)-(sensorLine.p1.y-sensorLine.p2.y)*(wall.p1.x-wall.p2.x));
                    iy = ((sensorLine.p1.x*sensorLine.p2.y-sensorLine.p1.y*sensorLine.p2.x)*(wall.p1.y-wall.p2.y)-(sensorLine.p1.y-sensorLine.p2.y)*(wall.p1.x*wall.p2.y-wall.p1.y*wall.p2.x))/((sensorLine.p1.x-sensorLine.p2.x)*(wall.p1.y-wall.p2.y)-(sensorLine.p1.y-sensorLine.p2.y)*(wall.p1.x-wall.p2.x));

                    //~ std::cout << "    The wall (" << wall.p1.x << "," << wall.p1.y << ") ->  (" << wall.p2.x << "," << wall.p2.y << ") intersects at (" << ix << "," << iy << ")" << std::endl;

                    if(wall.p1.x < wall.p2.x) {
                        minx = wall.p1.x;
                        maxx = wall.p2.x;
                    } else {
                        maxx = wall.p1.x;
                        minx = wall.p2.x;
                    }

                    if(wall.p1.y < wall.p2.y) {
                        miny = wall.p1.y;
                        maxy = wall.p2.y;
                    } else {
                        maxy = wall.p1.y;
                        miny = wall.p2.y;
                    }

                    if(((ix + e >= minx) && (ix - e <= maxx)) && ((iy + e >= miny) && (iy - e <= maxy))) {
                        //~ std::cout << "    I found intersection: " << ix << ", " << iy << std::endl;
                        if(((iy - perpSensorLine.p1.y) * (perpSensorLine.p2.x - perpSensorLine.p1.x) - (ix - perpSensorLine.p1.x) * (perpSensorLine.p2.y - perpSensorLine.p1.y)) > 0) {
                            //~ std::cout << "         and it was valid." << std::endl;
                            dist = std::sqrt(SQUARE(iy - state[states::y]) + SQUARE(ix - state[states::x]));
                            if(dist < shortestDist) {
                                shortestDist = dist;
                            }
                        }
                        //~ else {
                            //~ std::cout << "         but it was invalid." << std::endl;
                        //~ }
                    }
                }

                //~ std::cout << "End of loop" << std::endl;

                return shortestDist;
            }
        };
    }
}

#endif
