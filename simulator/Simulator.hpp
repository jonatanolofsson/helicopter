#ifndef SYS_SIMULATOR_HPP_
#define SYS_SIMULATOR_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/simulator/API.hpp>

namespace sys {
    namespace simulator {
        template<typename MotionModel_, typename FilterState_>
        class Simulator {
            public:
                typedef MotionModel_ MotionModel;
                typedef FilterState_ FilterState;
                typedef typename FilterState::Controls ControlState;
                typedef typename FilterState::States SystemState;
                typedef Simulator<MotionModel, FilterState> Self;

            private:
                os::Dispatcher<Self, ControlState> dispatcher;
                FilterState state;

            public:
                Simulator();

                void simulate(const ControlState u);
        };
    }
}

#endif
