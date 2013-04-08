#include <sys/particlefilter/API.hpp>

namespace sys {
    namespace particlefilter {
        Sensor sensor{{
            {Sensor::SensorType::Position(0.1, 0), 0},
            {Sensor::SensorType::Position(0.0, 0.1), M_PI_2},
            {Sensor::SensorType::Position(-0.1, 0), M_PI},
            {Sensor::SensorType::Position(0.0, -0.1), -M_PI_2}
        },
        {{
            // border
            {{0.00, 0.00}, {0.00, 2.40}},
            {{0.00, 2.40}, {2.40, 2.40}},
            {{2.40, 2.40}, {2.40, 0.00}},
            {{2.40, 0.00}, {0.00, 0.00}},

            // vertical walls
            {{0.69, 1.51}, {0.69, 2.40}},
            {{0.69, 0.46}, {0.69, 1.01}},
            {{1.17, 0.00}, {1.17, 0.46}},
            {{1.17, 1.37}, {1.17, 1.94}},
            {{1.94, 1.37}, {1.94, 1.94}},

            // horizontal walls
            {{1.17, 1.94}, {1.94, 1.94}},
            {{0.46, 1.51}, {0.69, 1.51}},
            {{1.65, 1.37}, {1.94, 1.37}},
            {{0.00, 1.01}, {0.69, 1.01}},
            {{1.17, 0.89}, {2.40, 0.89}}
        }}};

        ParticleFilter::ParticleFilter()
        : controlDispatcher(&Self::timeUpdate, this)
        , sensorDispatcher(&Self::measurementUpdate, this)
        {
            StateDescription::initialize(filter);
            for(auto& p : filter.particles) {
                p.state.setZero();
                p.weight = 1.0/nofParticles;
            }
            filter.maxWeight = 1.0/nofParticles;

            m.sensor = &sensor;
            m.R.setIdentity();
            m.R *= 0.1;
        }

        void ParticleFilter::timeUpdate(const ControlMessage u) {
            Algorithm::propagate<MotionModel>(filter, u, dT);
        }
        void ParticleFilter::measurementUpdate(const SensorMessage s) {
            m.z << s.distance[0], s.distance[1], s.distance[2], s.distance[3];
            Algorithm::measurementUpdate<>(filter, m);
        }
    }
}
