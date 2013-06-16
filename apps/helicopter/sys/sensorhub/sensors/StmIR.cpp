#include <sys/sensorhub/sensors/StmIR.hpp>
#include <sys/math/filtering.hpp>
#include <sys/math/models.hpp>
#include <os/com/getSignal.hpp>
#include <sys/States.hpp>
#include <sys/global.hpp>

#include <iostream>

INSTANTIATE_SIGNAL(sys::sensorhub::stmir::DistanceMeasurement);

namespace sys {
    namespace sensorhub {
        stmir::Sensor sensor{{
            {stmir::Sensor::SensorType::Position(0.1, 0), 0},
            {stmir::Sensor::SensorType::Position(0.0, 0.1), M_PI_2},
            {stmir::Sensor::SensorType::Position(-0.1, 0), M_PI},
            {stmir::Sensor::SensorType::Position(0.0, -0.1), -M_PI_2}
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

        StmIR::StmIR() : d(&StmIR::handleMessage, this) {
            m.sensor = &sensor;
            m.R.setIdentity();
            m.R *= 0.1;
        }

        void StmIR::handleMessage(const stm::SensorMessage s) {
            Scalar distanceScaling = 0.001;
            std::cout << "Got sensormessage: " << s.distance[0] << ", " <<  s.distance[1] << ", " <<  s.distance[2] << ", " <<  s.distance[3] << std::endl;
            m.z <<
                distanceScaling * s.distance[0],
                distanceScaling * s.distance[1],
                distanceScaling * s.distance[2],
                distanceScaling * s.distance[3];

            //~ std::stringstream cmsg; cmsg << "DISTANCE=[" << m.z[0] << "," << m.z[1] << "," << m.z[2] << "," << m.z[3] << "]\n";
            //~ debugServer.sendString(cmsg.str());
        }
    }
}
