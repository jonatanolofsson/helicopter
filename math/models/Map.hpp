#pragma once
#ifndef SYS_MATH_MODELS_MAP_HPP_
#define SYS_MATH_MODELS_MAP_HPP_

#include <limits>

namespace sys {
    namespace math {
        template<typename ParticleType, unsigned N>
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

            Matrix<Scalar, N, 1> getIntersection(const ParticleType& state, const Scalar sensorAngle) {
                typedef ParticleState states;
                Scalar ix;
                Scalar iy;
                Scalar dist;
                Scalar shortestDist = std::numeric_limits::<Scalar>::max();
                Scalar angle = state[states::heading] + sensorAngle

                Line sensorLine = {{state[states::x], state[states::y]}, {state[states::x] + std::cos(angle), state[states:y] + std::sin(angle)}};
                Line perpSensorLine = {{state[states::x], state[states::y]}, {sensorLine.p2.y - sensorLine.p1.y + sensorLine.p1.x, sensorLine.p1.x - sensorLine.p2.x + sensorLine.p1.y}};
                static const Scalar e = 1e-10;
                Scalar pX = (state[states::x] )
                for(Wall& wall : walls) {
                    ix = ((sensorLine.p1.x*sensorLine.p2.y-sensorLine.p1.y*sensorLine.p2.x)*(wall.p1.x-wall.p2.x)-(sensorLine.p1.x-sensorLine.p2.x)*(wall.p1.x*wall.p2.y-wall.p1.y*wall.p2.x))/((sensorLine.p1.x-sensorLine.p2.x)*(wall.p1.y-wall.p2.y)-(sensorLine.p1.y-sensorLine.p2.y)*(wall.p1.x-wall.p2.x));
                    iy = ((sensorLine.p1.x*sensorLine.p2.y-sensorLine.p1.y*sensorLine.p2.x)*(wall.p1.y-wall.p2.y)-(sensorLine.p1.y-sensorLine.p2.y)*(wall.p1.x*wall.p2.y-wall.p1.y*wall.p2.x))/((sensorLine.p1.x-sensorLine.p2.x)*(wall.p1.y-wall.p2.y)-(sensorLine.p1.y-sensorLine.p2.y)*(wall.p1.x-wall.p2.x));

                    if(((ix + e >= wall.p1.x) && (ix - e <= wall.p2.x)) && ((iy + e >= wall.p1.y) && (iy - e <= wall.p2.y))) {
                        if(((iy - perpSensorLine.p1.y) * (perpSensorLine.p2.x - perpSensorLine.p1.x) - (ix - perpSensorLine.p1.x) * (perpSensorLine.p2.y - perpSensorLine.p1.y)) > 0) {
                            dist = std::sqrt(SQUARE(iy - state[states::y]) + SQUARE(ix - state[states::x]));
                            if(dist < shortestDist) {
                                shortestDist = dist;
                            }
                        }
                    }
                }

                return shortestDist;
            }
        };
    }
}

#endif
