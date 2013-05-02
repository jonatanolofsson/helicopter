#pragma once
#ifndef SYS_SIMULATOR_HPP_
#define SYS_SIMULATOR_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/simulator/API.hpp>
#include <sys/motioncontrol/API.hpp>

namespace sys {
    namespace simulator {
        template<typename MotionModel_, typename Filter_>
        class Simulator {
            public:
                typedef ModelDescription::Controls Controls;
                typedef MotionModel_ MotionModel;
                typedef Filter_ Filter;
                typedef Simulator<MotionModel, Filter> Self;

            private:
                os::Dispatcher<Self, Controls> controlDispatcher;
                Filter filter;

            public:
                Simulator();

                void simulate(const Controls u);
        };
    }
}

#endif
