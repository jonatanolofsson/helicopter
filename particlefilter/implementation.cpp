#include <sys/particlefilter/API.hpp>

namespace sys {
    namespace particlefilter {
        ParticleFilter::ParticleFilter()
        : controlDispatcher(&Self::timeUpdate, this)
        , sensorDispatcher(&Self::measurementUpdate, this)
        {
            StateDescription::initialize(filter);
            for(auto& p : filter.particles) {
                p.state << sys::math::randU()*2.2 + 0.1, sys::math::randU()*2.2 + 0.1, sys::math::randU()*2*M_PI;
                p.weight = 1.0/nofParticles;
            }
            filter.maxWeight = 1.0/nofParticles;
        }

        void ParticleFilter::timeUpdate(const ControlMessage u) {
            Algorithm::propagate<MotionModel>(filter, u, dT);
        }

        void ParticleFilter::measurementUpdate(const SensorMessage m) {
            Algorithm::measurementUpdate<>(filter, m);
        }
    }
}
