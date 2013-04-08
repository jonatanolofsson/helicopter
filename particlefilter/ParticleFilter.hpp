#pragma once
#ifndef SYS_PARTICLEFILTER_HPP_
#define SYS_PARTICLEFILTER_HPP_

#include <algorithm>
#include <sys/math/models.hpp>
#include <sys/math/models/Map.hpp>
#include <sys/math/filtering.hpp>
#include <sys/math/states.hpp>
#include <sys/math/statistics.hpp>
#include <Eigen/Core>

namespace sys {
    namespace particlefilter {
        typedef sys::math::models::UltrasoundInMap<ModelDescription, 4, sys::math::Map<14>> Sensor;

        struct ParticleFilter {
                typedef ParticleFilter Self;
                os::Dispatcher<ParticleFilter, ControlMessage> controlDispatcher;
                os::Dispatcher<ParticleFilter, SensorMessage> sensorDispatcher;

                sys::math::GaussianMeasurement<Sensor, true> m;

                ParticleFilter();

                void timeUpdate(const ControlMessage u);
                void measurementUpdate(const SensorMessage s);

                Filter filter;
        };
    }
}

#endif
