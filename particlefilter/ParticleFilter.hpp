#pragma once
#ifndef SYS_PARTICLEFILTER_HPP_
#define SYS_PARTICLEFILTER_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/particlefilter/API.hpp>

namespace sys {
    namespace particlefilter {
        struct ParticleFilter {
            typedef ParticleFilter Self;
            os::Dispatcher<ParticleFilter, ControlMessage> controlDispatcher;
            os::Dispatcher<ParticleFilter, SensorMessage> sensorDispatcher;

            ParticleFilter();

            void timeUpdate(const ControlMessage u);
            void measurementUpdate(const SensorMessage s);

            Filter filter;
        };
    }
}

#endif
