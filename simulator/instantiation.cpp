#include <sys/simulator/API.hpp>
#include <sys/simulator/implementation.hpp>

namespace sys {
    namespace simulator {
        PfSensor pfSensor{{
            {PfSensor::SensorType::Position(0.1, 0), 0},
            {PfSensor::SensorType::Position(0.0, 0.1), M_PI_2},
            {PfSensor::SensorType::Position(-0.1, 0), M_PI},
            {PfSensor::SensorType::Position(0.0, -0.1), -M_PI_2}
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

        template class Simulator<MotionModel, Filter>;
    }
}
