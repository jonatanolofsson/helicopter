#pragma once
#ifndef SYS_MATH_FILTERING_PF_HPP_
#define SYS_MATH_FILTERING_PF_HPP_

#include <sys/math/filtering/Particle.hpp>
#include <sys/math/filtering/ParticleFilter.hpp>
#include <sys/math/statistics.hpp>
#include <iostream>

namespace sys {
    namespace math {
        struct PF {
            private:
                template<typename Filter>
                static void resample(Filter& filter) {
                    Scalar beta = 0;
                    unsigned index = irandU<Filter::nofParticles>();

                    // Wheel of resampling
                    for(auto& p : filter.particles) {
                        beta += randU() * 2 * filter.maxWeight;
                        while(filter.particles[index].weight < beta) {
                            beta -= filter.particles[index].weight;
                            index = (index + 1) % Filter::nofParticles;
                        }
                        p.state = filter.particles[index].state;
                    }
                }

            public:
                template<typename MotionModel, typename Filter>
                static void propagate(Filter& filter, typename MotionModel::Controls& u, typename Filter::Scalar dT) {
                    resample(filter);
                    for(auto& p : filter.particles) {
                        p.state = MotionModel::predict(p.state, u, dT) + math::normalSample(MotionModel::covariance(dT));
                    }
                }
                template<typename MotionModel, typename Filter>
                static void propagate(Filter& filter, typename Filter::Scalar dT) {
                    resample(filter);
                    for(auto& p : filter.particles) {
                        p.state = MotionModel::predict(p.state, dT) + math::normalSample(MotionModel::covariance(dT));
                    }
                }

                template<typename Filter, typename Measurement>
                static void measurementUpdate(Filter& filter, const Measurement& m) {
                    Scalar weightSum = 0;
                    for(auto& p : filter.particles) {
                        p.weight = math::normalProbabilityUnscaled(Measurement::Sensor::measurement(p.state), m.z, m.R);
                        weightSum += p.weight;
                    }
                    Scalar scalingFactor = 1.0 / weightSum;
                    filter.maxWeight = 0;
                    filter.state.setZero();
                    for(auto& p : filter.particles) {
                        p.weight *= scalingFactor;
                        if(p.weight > filter.maxWeight) {
                            filter.maxWeight = p.weight;
                        }
                        filter.state += p.weight * p.state;
                    }
                }

                template<typename MotionModel, typename Filter>
                static void timeUpdate(Filter& filter, typename MotionModel::Controls& u, typename Filter::Scalar dT) {
                    static bool w = true; if(w) { std::cerr << "Warning: Particle filter time update should not be used. This is for testing purposes only." << std::endl; w = false; }
                    propagate<MotionModel>(filter, u, dT);
                    filter.state.setZero();
                    for(const auto& p : filter.particles) {
                        filter.state += p.weight * p.state;
                    }
                }

                template<typename MotionModel, typename Filter>
                static void timeUpdate(Filter& filter, typename Filter::Scalar dT) {
                    static bool w = true; if(w) { std::cerr << "Warning: Particle filter time update should not be used. This is for testing purposes only." << std::endl; w = false; }
                    propagate<MotionModel>(filter, dT);
                    filter.state.setZero();
                    for(const auto& p : filter.particles) {
                        filter.state += p.weight * p.state;
                    }
                }

                template<typename MotionModel, typename Filter, typename Measurement>
                static void update(Filter& filter, const Measurement& m, typename Filter::Scalar dT) {
                    propagate<MotionModel>(filter, dT);
                    measurementUpdate<Filter, Measurement::Sensor>(filter, m);
                }
        };
    }
}

#endif
