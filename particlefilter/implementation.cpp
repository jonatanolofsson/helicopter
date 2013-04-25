#include <sys/particlefilter/API.hpp>
#include <sys/states/API.hpp>
#include <os/com/getSignal.hpp>

INSTANTIATE_SIGNAL(sys::particlefilter::PfState);

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
            static int n = 0;
            n = (n + 1) % 5;
            if(n == 0) {
                Algorithm::propagate<MotionModel>(filter, u, dT);
            }
        }

        void ParticleFilter::measurementUpdate(const SensorMessage m) {
            Algorithm::measurementUpdate<>(filter, m);
            os::yield(PfState{filter.state});

            Scalar convergence = 0;
            for(auto& p : filter.particles) {
                convergence += (filter.state.segment<2>(0) - p.state.segment<2>(0)).norm() * p.weight;
            }
            std::cout << "Convergence value: " << convergence << std::endl;
            if(convergence < convergenceCriteria) {
                postEvent(events::GotGlobalPosition());
            }
        }
    }
}
